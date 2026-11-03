"""Frozen GEN59 failures and bounded integer-neighbour recovery contracts.

Recorded solver results isolate the new search from numerical solver timing;
the actual URDF and unchanged predictor still certify every real endpoint.
Synthetic FK is used only for control-flow and joint-boundary tests below.
Explicit copies of one recording also bracket a real SDK-cell boundary under
the current 10 mm request; these are constructed branch tests, not recordings
of a successful grasp or evidence of a robust physical arrival margin.
No result authorizes a path, a hardware response, or reuse of stale feedback.
"""
from copy import deepcopy
import hashlib
from itertools import product as cartesian_product
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from alicia_flexible_grasp.robot import held_pregrasp_selection as module
from alicia_flexible_grasp.robot.held_axes_prediction import HeldAxesPredictionError
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SerialUrdfFk, SDK_QUANTUM_RAD as Q

GUIDE = 'PREGRASP_HELD_AXES_GUIDE_DOES_NOT_REACH_FIXED_GOAL'
FIXTURE_SHA = 'bdae6b6347322bbcc92c495f4e175f3da3860c1d9d559a5cf00960b83d041e97'
MODEL_SHA = '7812d6b69e854ed9f37b4e3cbd813ecd53dab3504065b2d04d287abc6fd852e0'
FREE = [0, 2, 3, 5]


@pytest.fixture(scope='module')
def recorded():
    raw = (Path(__file__).parent/'fixtures/gen59_held_lattice_inputs.json').read_bytes()
    assert hashlib.sha256(raw).hexdigest() == FIXTURE_SHA
    data = json.loads(raw)
    assert data['model_sha256'] == MODEL_SHA and len(data['rows']) == 4
    xml = (Path(__file__).resolve().parents[3]/
        'deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/'
        'alicia_expanded_metric_candidate.urdf').read_text()
    assert hashlib.sha256(xml.encode()).hexdigest() == MODEL_SHA
    return data, xml


def call(fk, row, **kwargs):
    return module.select_held_pregrasp_endpoint(fk, row['sample'],
        row['pregrasp_goal'], row['approach_goal'], row['nominal_command_rad'],
        position_tolerance_m=row['position_tolerance_m'],
        orientation_tolerance_rad=row['orientation_tolerance_rad'], **kwargs)


def replay_solvers(monkeypatch, row, mutate=None):
    calls = []
    def solve(fun, seed, **kwargs):
        index = len(calls); assert index < 3
        trial = deepcopy(row['original_trials'][index])
        x = np.array(trial['solver_endpoint'], dtype=float)
        if mutate is not None: mutate(index, x)
        # The third original solver operates in radians for the free guide.
        if index == 2: x[6:] *= Q
        calls.append(index)
        return NS(x=x, success=trial['solver_success'], nit=trial['iterations'],
                  message=trial['solver_message'])
    monkeypatch.setattr(module, 'minimize', solve)
    return calls


def independent_error(fk, joints, goal):
    # Quaternion expansion independent of production pose_matrix/residual.
    x,y,z,w = goal[3:]; length = math.sqrt(x*x+y*y+z*z+w*w)
    x,y,z,w = [v/length for v in (x,y,z,w)]
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    actual = fk(np.asarray(joints))
    return (np.linalg.norm(actual[:3,3]-np.asarray(goal[:3])),
            math.acos(float(np.clip((np.trace(rotation.T@actual[:3,:3])-1)/2, -1, 1))))


def derived_lattice_boundary(fk, raw_row, *, extra_cells=0):
    """Construct a documented SDK-cell boundary without changing the recording.

    With trial 0 as origin, Joint2 +9 cells qualifies and +10 cells gives a
    10.528 mm approach residual; an adjacent lattice endpoint gives 9.998 mm.
    The copied pose0 translation follows the +10-cell centre FK (7.63 mm from
    the recorded goal). Its orientation and the entire approach goal stay
    fixed. Extra +2/+4 cells put every fully checked neighbour outside 10 mm.
    The three identical solver proposals below are explicit test providers,
    not a claim that the historical optimizer returned these endpoints.
    """
    before = deepcopy(raw_row)
    row = deepcopy(raw_row)
    offset = np.asarray(row['sample']['sdk_positions_rad'])-row['sample']['accepted_positions_rad']
    centre = np.array(row['original_trials'][0]['quantized_command_rad'])
    centre[1] += 10*Q
    row['pregrasp_goal'][:3] = fk(centre-offset)[:3,3].tolist()
    translation = np.asarray(row['pregrasp_goal'][:3])-raw_row['pregrasp_goal'][:3]
    centre[1] += extra_cells*Q
    row['nominal_command_rad'] = centre.tolist()
    row['position_tolerance_m'] = .010
    _, pre = module.predict_held_axes_endpoint(fk, centre, centre-offset,
        row['pregrasp_goal'], position_tolerance_m=.010,
        orientation_tolerance_rad=row['orientation_tolerance_rad'], allow_already_converged=True)
    assert pre['steps_predicted'] == 0
    with pytest.raises(HeldAxesPredictionError, match=GUIDE) as caught:
        module.predict_held_axes_endpoint(fk, pre['predicted_command_positions_rad'],
            pre['predicted_positions_rad'], row['approach_goal'], position_tolerance_m=.010,
            orientation_tolerance_rad=row['orientation_tolerance_rad'], allow_already_converged=True)
    rejected = caught.value.evidence
    pe, ae = independent_error(fk, rejected['predicted_positions_rad'], row['approach_goal'])
    assert pe > .010 and ae <= row['orientation_tolerance_rad']
    x = np.r_[centre-offset, np.asarray(rejected['proposed_delta_counts'])[FREE]]
    row['original_trials'] = [dict(solver_endpoint=x.tolist(), solver_success=False,
        iterations=1, solver_message='explicit derived SDK-cell boundary provider',
        quantized_command_rad=centre.tolist(), rejection=GUIDE,
        evidence=deepcopy(rejected)) for _ in range(3)]
    row['test_input_derivation'] = dict(source_fixture_sha256=FIXTURE_SHA,
        source_row=0, source_trial=0, joint2_offset_counts=10+extra_cells,
        pose0_translation_m=translation.tolist(), approach_goal_unchanged=True,
        historical_solver_results_replayed=False, branch_coverage_only=True)
    assert raw_row == before
    return row


@pytest.mark.parametrize('index', range(4))
def test_recorded_inputs_qualify_in_requested_window_with_true_residuals(recorded, monkeypatch, index):
    data, xml = recorded; row = deepcopy(data['rows'][index]); before = deepcopy(row)
    fk = SerialUrdfFk(xml, ARM_NAMES); calls = replay_solvers(monkeypatch, row)
    assert row['original_code'] == GUIDE
    assert all(t['rejection'] == GUIDE for t in row['original_trials'])
    command, report = call(fk, row)
    assert report['success']
    for name,goal,tolerance in [('predicted_pregrasp',row['pregrasp_goal'],.010),
                                ('predicted_approach',row['approach_goal'],.010)]:
        item = report[name]
        pe, ae = independent_error(fk, item['predicted_positions_rad'], goal)
        assert item['position_tolerance_m'] == tolerance
        assert pe <= tolerance and ae <= math.radians(5)
        np.testing.assert_allclose([pe,ae], [item['predicted_position_error_m'],
            item['predicted_orientation_error_rad']], atol=1e-12)
        delta = np.asarray(item['proposed_delta_counts'])
        assert delta[1] == delta[4] == 0 and np.max(abs(delta)) <= 32
    np.testing.assert_allclose(command/Q,np.rint(command/Q),atol=1e-12)
    if report['reselected']:
        assert calls
    else:
        assert calls == []
        np.testing.assert_array_equal(command, report['original_nominal_command_rad'])
    assert row == before
    assert report['fixed_pregrasp_goal'] == row['pregrasp_goal']
    assert report['fixed_approach_goal'] == row['approach_goal']
    assert report['source_sdk_stamp_ns'] == row['sample']['sdk_stamp_ns']
    assert report['source_accepted_stamp_ns'] == row['sample']['accepted_stamp_ns']
    assert report['epoch_ns'] == row['sample']['epoch_ns']
    assert all(report[k] is False for k in ('execution_authorized','trajectory_checked',
        'future_following_verified','hardware_response_verified','globally_optimal'))
    json.dumps(report, allow_nan=False)


def assert_lattice_trace(row, report):
    """Keep the complete lattice invariants only where the lattice really ran."""
    audit = report['quantized_neighborhood']
    assert audit['eligible_seed_indices'] == [0,1,2]
    assert audit['enumerated_count'] == audit['maximum_enumerated_count'] == 2187
    assert audit['unique_candidate_count']+audit['duplicate_count'] == 2187
    assert 1 <= len(audit['checked']) <= audit['max_full_checks'] == 16
    assert len({tuple(c['sdk_counts']) for c in audit['checked']}) == len(audit['checked'])
    ordered = [(c['ranking_proxy_cost'],tuple(c['sdk_counts'])) for c in audit['checked']]
    assert ordered == sorted(ordered)
    for c in audit['checked']:
        centre = module.sdk_counts(row['original_trials'][c['seed_index']]['quantized_command_rad'])
        assert np.max(abs(np.asarray(c['sdk_counts'])-centre)) <= 1
    json.dumps(report, allow_nan=False)


def test_explicit_strict_request_rejects_all_neighbours_at_pregrasp_screen(recorded, monkeypatch):
    data, xml = recorded
    row = deepcopy(data['rows'][2])
    row['position_tolerance_m'] = .006
    fk = SerialUrdfFk(xml, ARM_NAMES)
    calls = replay_solvers(monkeypatch, row)
    with pytest.raises(HeldAxesPredictionError, match=GUIDE) as caught:
        call(fk, row)
    report = caught.value.evidence['pregrasp_endpoint_selection']
    assert calls == [0, 1, 2]
    audit = report['quantized_neighborhood']
    assert audit['eligible_seed_indices'] == [0, 1, 2]
    assert audit['enumerated_count'] == audit['maximum_enumerated_count'] == 2187
    assert audit['unique_candidate_count']+audit['duplicate_count'] == 2187
    assert audit['pregrasp_screen_pass_count'] == 0
    assert audit['checked'] == [] and audit['success'] is False
    assert report['execution_authorized'] is False


@pytest.mark.parametrize('extra_cells', [2, 4])
def test_derived_boundary_exhausts_real_sixteen_pair_checks(recorded, monkeypatch, extra_cells):
    data, xml = recorded
    fk = SerialUrdfFk(xml, ARM_NAMES)
    row = derived_lattice_boundary(fk, data['rows'][0], extra_cells=extra_cells)
    calls = replay_solvers(monkeypatch, row)
    with pytest.raises(HeldAxesPredictionError, match=GUIDE) as caught:
        call(fk, row)
    report = caught.value.evidence['pregrasp_endpoint_selection']
    assert calls == [0, 1, 2]
    assert_lattice_trace(row, report)
    audit = report['quantized_neighborhood']
    assert audit['pregrasp_screen_pass_count'] > len(audit['checked'])
    assert audit['success'] is False and len(audit['checked']) == 16
    for item in audit['checked']:
        assert item['qualified'] is False
        rejected = item['rejection_evidence']
        assert rejected['fixed_goal_pose'] in (row['pregrasp_goal'], row['approach_goal'])
        pe, ae = independent_error(fk, rejected['predicted_positions_rad'], rejected['fixed_goal_pose'])
        np.testing.assert_allclose([pe, ae], [rejected['predicted_position_error_m'],
            rejected['predicted_orientation_error_rad']], atol=1e-12)
        assert pe > rejected['position_tolerance_m'] or ae > row['orientation_tolerance_rad']


class SyntheticFk(SerialUrdfFk):
    """Constant FK tests filtering/flow only, never physical qualification."""
    def __init__(self, limits=(-4.,4.)):
        self.names = ARM_NAMES; self.model_sha256 = 'synthetic-flow-model'
        self.limits = np.tile(limits,(6,1)); self.on_call = None; self.seen = []
    def __call__(self,q):
        q = np.asarray(q); assert q.shape == (6,) and np.all(np.isfinite(q))
        if self.on_call is not None: self.on_call(q)
        self.seen.append(q.copy()); return np.eye(4)


def synthetic(monkeypatch, *, nominal=0., offset=0., limits=(-4.,4.), endpoint=None):
    fk = SyntheticFk(limits); q = np.full(6,nominal); bias = np.full(6,offset)
    row = dict(sample=dict(model_sha256=fk.model_sha256,joint_names=list(ARM_NAMES),
        sdk_positions_rad=q.tolist(),accepted_positions_rad=(q-bias).tolist()),
        pregrasp_goal=[0,0,0,0,0,0,1],approach_goal=[0,0,0,0,0,0,1],
        nominal_command_rad=q.tolist(),position_tolerance_m=.010,
        orientation_tolerance_rad=math.radians(5))
    x = np.r_[q-bias if endpoint is None else np.asarray(endpoint),np.zeros(4)]
    row['original_trials'] = [dict(solver_endpoint=x.tolist(),solver_success=False,
        iterations=1,solver_message='controlled boundary provider') for _ in range(3)]
    state = dict(lattice=False,predictor_calls=0,lattice_calls=0,products=0)
    replay_solvers(monkeypatch,row)
    def product(*args,**kwargs):
        state['lattice'] = True; state['products'] += 1
        yield from cartesian_product(*args,**kwargs)
    monkeypatch.setattr(module,'product',product)
    def rejected(*args,**kwargs):
        state['predictor_calls'] += 1
        if state['lattice']: state['lattice_calls'] += 1
        raise HeldAxesPredictionError(GUIDE,dict(proposed_delta_counts=[0]*6,code=GUIDE))
    monkeypatch.setattr(module,'predict_held_axes_endpoint',rejected)
    return fk,row,state


def failed_report(fk,row,**kwargs):
    with pytest.raises(HeldAxesPredictionError,match=GUIDE) as caught: call(fk,row,**kwargs)
    return caught.value.evidence['pregrasp_endpoint_selection']


def test_deduplicated_first_seed_proxy_order_and_exact_sixteen_pair_budget(monkeypatch):
    fk,row,state = synthetic(monkeypatch)
    report = failed_report(fk,row); a = report['quantized_neighborhood']
    assert state['products'] == 3 and a['enumerated_count'] == 2187
    assert a['unique_candidate_count'] == a['pregrasp_screen_pass_count'] == 729
    assert a['duplicate_count'] == 1458 and len(a['checked']) == 16
    assert state['lattice_calls'] == 16  # Every pregrasp rejects before the paired approach.
    assert all(v['seed_index'] == 0 for v in a['checked'])
    keys = [tuple(v['sdk_counts']) for v in a['checked']]
    assert keys == list(cartesian_product((2047,2048,2049),repeat=6))[:16]
    assert a['success'] is False and a['pregrasp_screen_pass_count'] > len(a['checked'])
    assert not a['execution_authorized'] and not a['trajectory_checked']
    json.dumps(report,allow_nan=False)


@pytest.mark.parametrize('kind', ['local_low','local_high','urdf_low','urdf_high','sdk_low','sdk_high'])
def test_joint_filter_checks_actual_bias_and_sdk_and_urdf_bounds(monkeypatch,kind):
    nominal,offset,limits = 0.,.020,(-4.,4.)
    if kind.startswith('urdf'): limits = (-.2,.2)
    if kind.startswith('sdk'): nominal = -math.pi+.1 if kind.endswith('low') else math.pi-.1
    nominal_q = (module.sdk_counts(np.full(6,nominal))-2048)*Q
    low = np.maximum.reduce((nominal_q-.25,np.full(6,limits[0]),np.full(6,limits[0]-offset),np.full(6,-math.pi-offset)))
    high = np.minimum.reduce((nominal_q+.25,np.full(6,limits[1]),np.full(6,limits[1]-offset),np.full(6,2047*Q-offset)))
    boundary = low if kind.endswith('low') else high
    # Place the continuous centre at the active bound; outward lattice cells
    # must be filtered before FK, even when the SDK command alone is valid.
    fk,row,state = synthetic(monkeypatch,nominal=nominal,offset=offset,limits=limits,endpoint=boundary)
    seen = []
    def observe(q):
        if state['lattice']:
            assert np.all(q >= low-1e-14) and np.all(q <= high+1e-14)
            seen.append(q.copy())
    fk.on_call = observe
    report = failed_report(fk,row); a = report['quantized_neighborhood']
    assert seen and 0 < a['pregrasp_screen_pass_count'] < a['unique_candidate_count']
    for item in a['checked']:
        command = np.asarray(item['command_rad']); actual = command-offset
        assert np.all(actual >= low-1e-14) and np.all(actual <= high+1e-14)
        assert np.all(command >= limits[0]) and np.all(command <= limits[1])
        assert all(0 <= n <= 4095 for n in item['sdk_counts'])


@pytest.mark.parametrize('fault',['inf','outside_urdf'])
def test_nonfinite_or_urdf_mixed_trial_does_not_create_empty_fallback(monkeypatch,fault):
    fk,row,state = synthetic(monkeypatch,limits=(-2.,2.))
    def mutate(index,x):
        if index == 1: x[0] = float('inf') if fault == 'inf' else 2.1
    replay_solvers(monkeypatch,row,mutate)
    report = failed_report(fk,row)
    assert report['trials'][1]['rejection'] == ('NONFINITE_SOLVER_RESULT' if fault == 'inf'
        else 'QUANTIZED_COMMAND_OUTSIDE_URDF')
    assert state['products'] == 0 and 'quantized_neighborhood' not in report


def test_continuous_guide_outside_original_count_box_is_not_an_eligible_seed(monkeypatch):
    fk,row,state = synthetic(monkeypatch)
    def mutate(index,x): x[6] = 33.
    replay_solvers(monkeypatch,row,mutate)
    audit = failed_report(fk,row)['quantized_neighborhood']
    assert audit['eligible_seed_indices'] == [] and audit['enumerated_count'] == 0
    assert audit['checked'] == [] and state['products'] == state['lattice_calls'] == 0
    assert audit['success'] is False and audit['execution_authorized'] is False


@pytest.mark.parametrize('fault',['none','pregrasp_collision','held_curve_collision','held_curve_moves_fixed_axis','joint_plan_changed'])
def test_recovered_endpoint_still_enters_original_dual_curve_and_contact_gates(recorded,monkeypatch,fault):
    import test_held_pregrasp_selection as old
    from alicia_flexible_grasp.robot import contact_sequence_precheck as precheck
    data,xml = recorded; fk = SerialUrdfFk(xml,ARM_NAMES)
    row = derived_lattice_boundary(fk, data['rows'][0])
    _,case = old.recorded_case(); case.update(sample=deepcopy(row['sample']),nominal_command=row['nominal_command_rad'])
    case['poses'][:2] = [row['pregrasp_goal'],row['approach_goal']]
    monkeypatch.setattr(old,'recorded_case',lambda: (fk,deepcopy(case)))
    replay_solvers(monkeypatch,row)
    original = module.select_held_pregrasp_endpoint; seen = []
    def select(*args,**kwargs):
        command,report = original(*args,**kwargs)
        assert report['quantized_neighborhood']['success']
        assert_lattice_trace(row, report)
        for key, goal in [('predicted_pregrasp', row['pregrasp_goal']),
                          ('predicted_approach', row['approach_goal'])]:
            pe, ae = independent_error(fk, report[key]['predicted_positions_rad'], goal)
            assert pe <= .010 and ae <= row['orientation_tolerance_rad']
        assert report['trajectory_checked'] is False and report['execution_authorized'] is False
        seen.append(deepcopy(report)); return command,report
    monkeypatch.setattr(precheck,'select_held_pregrasp_endpoint',select)
    old.test_real_precheck_uses_selected_joint_start_and_audits_future_guide(
        monkeypatch, fault, position_tolerance_m=.010, expected_reselected=True)
    assert len(seen) == 1
