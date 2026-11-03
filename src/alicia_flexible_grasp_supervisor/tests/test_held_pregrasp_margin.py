"""Recorded arrival drift, independent FK residuals and refinement boundaries."""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from alicia_flexible_grasp.robot import held_pregrasp_margin as module
from alicia_flexible_grasp.robot.held_axes_prediction import predict_held_axes_endpoint, HeldAxesPredictionError
from alicia_flexible_grasp.robot.stationary_following import SerialUrdfFk, ARM_NAMES, SDK_QUANTUM_RAD as Q
from test_gen60_held_pregrasp_lattice import independent_error


def recorded():
    w = Path(__file__).resolve().parents[3]
    data = json.loads((Path(__file__).parent/'fixtures/contact_arrival_20261023.json').read_text())
    fk = SerialUrdfFk((w/'deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/'
        'alicia_expanded_metric_candidate.urdf').read_text(), ARM_NAMES)
    assert fk.model_sha256 == data['model_sha256']
    return fk, data


def call(fk, d, *, position_tolerance_m=.01, **kwargs):
    return module.refine_held_pregrasp_margin(fk, d['sample'], *d['poses'][:2],
        (np.asarray(d['command_endpoint_counts'])-2048)*Q,
        position_tolerance_m=position_tolerance_m, orientation_tolerance_rad=math.radians(5), **kwargs)


def sequence(fk, command, offset, poses, *, approach_tolerance_m=.010):
    _, pre = predict_held_axes_endpoint(fk, command, command-offset, poses[0],
        position_tolerance_m=.01, orientation_tolerance_rad=math.radians(5),
        allow_already_converged=True)
    _, approach = predict_held_axes_endpoint(fk, pre['predicted_command_positions_rad'],
        pre['predicted_positions_rad'], poses[1], position_tolerance_m=approach_tolerance_m,
        orientation_tolerance_rad=math.radians(5), allow_already_converged=True)
    return pre, approach


@pytest.mark.parametrize('yaw_counts', [-160, 0, 160])
def test_recorded_drift_passes_original_guides_after_refinement_without_future_sample(yaw_counts):
    fk, d = recorded()
    # Rigid base rotation changes both goals, source joint state and endpoint.
    # Future measured bias is withheld from the production optimizer.
    rot = Rotation.from_euler('z', yaw_counts*Q)
    for p in d['poses']:
        p[:3] = rot.apply(p[:3]).tolist()
        p[3:] = (rot*Rotation.from_quat(p[3:])).as_quat().tolist()
    d['command_endpoint_counts'][0] += yaw_counts
    for field in ('sdk_positions_rad', 'accepted_positions_rad'):
        d['sample'][field][0] += yaw_counts*Q
    before = deepcopy(d)
    old = (np.array(d['command_endpoint_counts'])-2048)*Q
    arrival_offset = np.array(d['arrival_sdk_minus_actual_counts'])*Q
    with pytest.raises(HeldAxesPredictionError, match='DOES_NOT_REACH_FIXED_GOAL'):
        sequence(fk, old, arrival_offset, d['poses'], approach_tolerance_m=.006)
    command, report = call(fk, d)
    assert report['refined'] and report['selected_normalized_cost'] < report['original_normalized_cost']
    offset = np.array(d['sample']['sdk_positions_rad'])-d['sample']['accepted_positions_rad']
    pe, ae = independent_error(fk, command-offset, d['poses'][0])
    assert pe < .006 and ae < math.radians(1)
    for proof, goal, tol in zip(sequence(fk, command, arrival_offset, d['poses'],
            approach_tolerance_m=.006), d['poses'], [.01,.006]):
        pe, ae = independent_error(fk, proof['predicted_positions_rad'], goal)
        assert pe <= tol and ae <= math.radians(5)
        delta = proof['proposed_delta_counts']
        assert max(abs(np.array(delta))) <= 32 and delta[1] == delta[4] == 0
    assert d == before
    np.testing.assert_allclose(command/Q, np.rint(command/Q), atol=1e-12)
    for field in ('future_following_verified','hardware_response_verified','execution_authorized',
                  'trajectory_checked','globally_optimal'):
        assert report[field] is False
    json.dumps(report, allow_nan=False)


@pytest.mark.parametrize('index', range(3))
def test_historical_different_poses_keep_feasibility_and_never_worsen_arrival_cost(index):
    from test_held_pregrasp_selection import recorded_case, select
    fk, d = recorded_case(index)
    original, _ = select(fk, d)
    command, report = module.refine_held_pregrasp_margin(fk, d['sample'], *d['poses'][:2],
        original, position_tolerance_m=.01, orientation_tolerance_rad=math.radians(5))
    offset = np.array(d['sample']['sdk_positions_rad'])-d['sample']['accepted_positions_rad']
    sequence(fk, command, offset, d['poses'])
    assert report['selected_normalized_cost'] <= report['original_normalized_cost']


@pytest.mark.parametrize('fault', ['model','names','nan','bias','relaxed_tolerance'])
def test_invalid_inputs_never_enter_optimizer(monkeypatch, fault):
    fk, d = recorded()
    if fault == 'model':d['sample']['model_sha256'] = 'unsupported'
    if fault == 'names':d['sample']['joint_names'] = list(reversed(ARM_NAMES))
    if fault == 'nan':d['sample']['sdk_positions_rad'][0] = float('nan')
    if fault == 'bias':d['sample']['sdk_positions_rad'][0] += .1
    optimizer = mock.Mock(side_effect=AssertionError('invalid input optimization'))
    monkeypatch.setattr(module,'minimize',optimizer)
    with pytest.raises(ValueError):
        if fault == 'relaxed_tolerance':
            module.refine_held_pregrasp_margin(fk, d['sample'], *d['poses'][:2],
                (np.array(d['command_endpoint_counts'])-2048)*Q,
                position_tolerance_m=.020001, orientation_tolerance_rad=math.radians(5))
        else:call(fk,d)
    optimizer.assert_not_called()


@pytest.mark.parametrize('where', ['objective','constraint','after_solver'])
def test_deadline_and_revocation_propagate_without_fallback(monkeypatch,where):
    fk,d=recorded(); cancelled=[False]
    def check():
        if cancelled[0]:raise RuntimeError('authority cancelled')
    def optimizer(fun,seed,**kwargs):
        cancelled[0]=True
        if where=='objective':fun(seed)
        if where=='constraint':kwargs['constraints'][0]['fun'](seed)
        return NS(x=seed, success=True, nit=1, message='synthetic')
    monkeypatch.setattr(module,'minimize',optimizer)
    with pytest.raises(RuntimeError,match='authority cancelled'):call(fk,d,deadline_check=check)


@pytest.mark.parametrize('fault',['nonfinite','out_of_bounds','no_improvement'])
def test_failed_preference_keeps_original_qualified_endpoint(monkeypatch,fault):
    fk,d=recorded()
    solver_calls=[]
    def optimizer(fun,seed,**kwargs):
        solver_calls.append(True)
        x=seed.copy()
        if fault=='nonfinite':x[0]=float('nan')
        if fault=='out_of_bounds':x[0]+=1.
        return NS(x=x,success=True,nit=1,message='synthetic solver success')
    monkeypatch.setattr(module,'minimize',optimizer)
    command,report=call(fk,d)
    assert solver_calls==[True]
    assert not report['refined']
    np.testing.assert_array_equal(module.sdk_counts(command),d['command_endpoint_counts'])


def test_recorded_pose_exact_preference_is_legal_at_ten_mm(monkeypatch):
    """The historical rejection proposal is now feasible; retain that input."""
    from scipy.optimize import least_squares
    from alicia_flexible_grasp.robot.pregrasp_compensation import pose_matrix
    fk, d = recorded(); before = deepcopy(d); calls = []
    def optimizer(fun, seed, **kwargs):
        calls.append(True)
        goal = pose_matrix(d['poses'][0])
        def residual(q):
            t = fk(q)
            return np.r_[(t[:3,3]-goal[:3,3])/.01,
                (goal[:3,:3].T@t[:3,:3]-np.eye(3)).reshape(-1)/.08]
        x = seed.copy(); x[:6] = least_squares(residual, seed[:6]).x
        return NS(x=x, success=True, nit=1, message='historical pose-exact test proposal')
    monkeypatch.setattr(module, 'minimize', optimizer)
    command, report = call(fk, d)
    assert calls == [True] and report['refined']
    offset = np.asarray(d['sample']['sdk_positions_rad'])-d['sample']['accepted_positions_rad']
    for proof, goal in zip(sequence(fk, command, offset, d['poses']), d['poses']):
        pe, ae = independent_error(fk, proof['predicted_positions_rad'], goal)
        assert pe <= .010 and ae <= math.radians(5)
    assert d == before
    # The same original endpoint fails a 6 mm request before any optimizer.
    calls.clear()
    with pytest.raises(HeldAxesPredictionError, match='DOES_NOT_REACH_FIXED_GOAL'):
        call(fk, d, position_tolerance_m=.006)
    assert calls == []


@pytest.mark.parametrize('pose0_distance_m', [.007, .008])
def test_better_pose0_preference_still_requires_real_approach_qualification(
        monkeypatch, tmp_path, pose0_distance_m):
    from test_gen60_held_pregrasp_lattice import (
        FIXTURE_SHA, derived_lattice_boundary, replay_solvers, call as select_boundary)
    fk, _ = recorded()
    raw = (Path(__file__).parent/'fixtures/gen59_held_lattice_inputs.json').read_bytes()
    assert hashlib.sha256(raw).hexdigest() == FIXTURE_SHA
    row = derived_lattice_boundary(fk, json.loads(raw)['rows'][0])
    selection_calls = replay_solvers(monkeypatch, row)
    selected, selected_report = select_boundary(fk, row)
    assert selection_calls == [0, 1, 2]
    assert selected_report['quantized_neighborhood']['success']
    offset = np.asarray(row['sample']['sdk_positions_rad'])-row['sample']['accepted_positions_rad']
    candidate = np.asarray(row['nominal_command_rad'])
    centre_pose, selected_pose = fk(candidate-offset), fk(selected-offset)
    displacement = selected_pose[:3,3]-centre_pose[:3,3]
    assert 0 < np.linalg.norm(displacement) < .001
    # Move only a copied pose0 goal away from the qualified neighbour. Both
    # endpoints remain inside 10 mm; the centre has a lower pose0 cost while
    # its unchanged approach still misses (10.528 vs the neighbour's 9.998 mm).
    # The 7/8 mm distances also keep the original cost above the optimizer's
    # .6**2 preference threshold. This is branch coverage, not grasp evidence.
    pose0 = deepcopy(row['pregrasp_goal'])
    pose0[:3] = (centre_pose[:3,3]-pose0_distance_m*displacement/np.linalg.norm(displacement)).tolist()
    rotation = (getattr(Rotation, 'from_matrix', None) or Rotation.from_dcm)(centre_pose[:3,:3])
    pose0[3:] = rotation.as_quat().tolist()
    d = dict(sample=deepcopy(row['sample']), poses=[pose0, deepcopy(row['approach_goal'])],
        command_endpoint_counts=module.sdk_counts(selected).tolist())
    before = deepcopy(d)
    sequence(fk, selected, offset, d['poses'])
    with pytest.raises(HeldAxesPredictionError, match='DOES_NOT_REACH_FIXED_GOAL'):
        sequence(fk, candidate, offset, d['poses'])
    def cost(command):
        p, a = independent_error(fk, command-offset, pose0)
        return (p/.010)**2+(a/math.radians(5))**2
    assert .6**2 < cost(candidate) < cost(selected)
    calls = []
    def optimizer(fun, seed, **kwargs):
        calls.append(True)
        x = seed.copy(); x[:6] = candidate-offset
        assert fun(x) < fun(seed)-1e-6
        assert all(low <= value <= high for value, (low, high) in zip(x, kwargs['bounds']))
        return NS(x=x, success=True, nit=1, message='explicit better-pose0 unqualified-approach proposal')
    monkeypatch.setattr(module, 'minimize', optimizer)
    command, report = call(fk, d)
    assert calls == [True] and not report['refined']
    assert report['reason'] == 'refinement_not_qualified'
    rejection = report['candidate_rejection']
    assert rejection['code'] == 'PREGRASP_HELD_AXES_GUIDE_DOES_NOT_REACH_FIXED_GOAL'
    proof = rejection['evidence']
    assert proof['fixed_goal_pose'] == d['poses'][1]
    pe, ae = independent_error(fk, proof['predicted_positions_rad'], d['poses'][1])
    assert pe > .010 and ae <= math.radians(5)
    np.testing.assert_array_equal(module.sdk_counts(command), d['command_endpoint_counts'])
    assert d == before
    (tmp_path/'derived_margin_branch.json').write_text(json.dumps(dict(
        derivation=row['test_input_derivation'], pose0_distance_m=pose0_distance_m,
        fixed_goals=d['poses'], original_normalized_cost=cost(selected),
        candidate_normalized_cost=cost(candidate), candidate_command_rad=candidate.tolist(),
        report=report), indent=2, allow_nan=False))


def test_already_central_endpoint_avoids_optimizer(monkeypatch):
    fk,d=recorded();command,_=call(fk,d)
    d['command_endpoint_counts']=module.sdk_counts(command).tolist()
    monkeypatch.setattr(module,'minimize',mock.Mock(side_effect=AssertionError('unnecessary refinement')))
    again,report=call(fk,d)
    np.testing.assert_array_equal(again,command)
    assert report['reason']=='already_inside_preferred_margin'


def test_gateway_checks_and_sends_refined_target_not_initial_selection(monkeypatch):
    from test_contact_pregrasp_gateway import case
    from alicia_flexible_grasp.robot import contact_pregrasp_gateway as gateway
    c=case(monkeypatch);target=c.target.copy();target[0]+=Q
    monkeypatch.setattr(gateway,'refine_held_pregrasp_margin',
        lambda *_a,**_k:(target,{'refined':True}))
    response=c.node.handle_contact_pregrasp(NS(plan=c.plan,execute=True))
    assert response.success,response.message
    np.testing.assert_array_equal(c.calls[0][0],target)
    report=json.loads(response.message)
    assert report['selection']['arrival_margin']['refined']
    assert report['complete_execution_path_checked'] and report['endpoint_confirmation_required']
    for args in c.curve.call_args_list:
        np.testing.assert_array_equal(args[0][0].joint_trajectory.points[-1].positions,target)


@pytest.mark.parametrize('fault',['collision','cancelled'])
def test_refinement_cannot_bypass_path_or_authority_rejection(monkeypatch,fault):
    from test_contact_pregrasp_gateway import case
    from alicia_flexible_grasp.robot import contact_pregrasp_gateway as gateway
    c=case(monkeypatch)
    if fault=='collision':c.curve.side_effect=ValueError('collision')
    else:monkeypatch.setattr(gateway,'refine_held_pregrasp_margin',mock.Mock(side_effect=ValueError('cancelled')))
    response=c.node.handle_contact_pregrasp(NS(plan=c.plan,execute=True))
    assert not response.success and fault in response.message
    assert c.state['moves']==0 and not c.calls


def test_actual_overlay_reaches_real_margin_prediction_with_fixed_sources():
    import yaml
    overlay = yaml.safe_load((Path(__file__).resolve().parents[2] /
        'alicia_grasp_modes/config/software_gate_relaxation.yaml').read_text())
    fk, d = recorded()
    before = deepcopy(d)
    command, report = module.refine_held_pregrasp_margin(
        fk, d['sample'], *d['poses'][:2],
        (np.asarray(d['command_endpoint_counts'])-2048)*Q,
        position_tolerance_m=overlay['/grasp/unknown_measured_endpoint_position_tolerance_m'],
        orientation_tolerance_rad=math.radians(overlay['/grasp/measured_endpoint_orientation_tolerance_deg']))
    assert report['predicted_pregrasp']['position_tolerance_m'] == .020
    assert report['predicted_approach']['position_tolerance_m'] == .012
    for key, goal, limit in [('predicted_pregrasp',d['poses'][0],.020),
                             ('predicted_approach',d['poses'][1],.012)]:
        proof=report[key]
        position, angle=independent_error(fk,proof['predicted_positions_rad'],goal)
        assert position <= limit and angle <= math.radians(10)
        assert proof['proposed_delta_counts'][1] == proof['proposed_delta_counts'][4] == 0
    assert report['execution_authorized'] is False and report['future_following_verified'] is False
    assert d == before
