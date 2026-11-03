"""Finite hints keep the measured center; fresh path gates retain authority."""
from copy import deepcopy
from dataclasses import replace
from functools import lru_cache
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

import alicia_flexible_grasp.grasp.contact_joint_placement as placement
from alicia_flexible_grasp.grasp.gripper_geometry import GRIPPER_CONTRACT_TOLERANCE_M
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from test_contact_joint_placement import (
    assert_hint, assert_fresh_endpoint_contract, bias_and_nominal, endpoint_checks,
    exact_same_branch_solution, fixture_case, geometry_only_clock, no_authority,
    serializable,
)

FIXTURE = Path(__file__).parent/'fixtures/contact_minimum_ray_terminal_20261103.json'


@lru_cache(maxsize=1)
def saved():
    value = json.loads(FIXTURE.read_text())
    assert value['execution_authorized'] is False
    assert hashlib.sha256(value['model_xml'].encode()).hexdigest() == value['model_sha256']
    return value


def case():
    return fixture_case(saved())


def direct_args(args, check=lambda: None):
    fk, command, seed = placement._failure_seed(args['failure'], args['plan_id'],
        args['source_stamp_ns'], args['goal_pose'], args['support_normal_base'], args['model_xml'])
    return dict(fk=fk, transforms=(args['contact_geometry']['planned_T_base_tool0'], command),
        seed=seed, bound=args['maximum_translation_m'], scene=args['scene'],
        gripper=args['contact_geometry']['gripper'], opening=args['failure']['opening_m'],
        geometry=args['contact_geometry'], limit=args['translation_evidence_limit_m'],
        solver_resolution_mm=placement.SOLVER_RESOLUTION_MM, check=check,
        maximum_evaluations=placement.MAXIMUM_EVALUATIONS)


@pytest.fixture(scope='module')
def solved():
    args = case()
    before = serializable(args)
    proposed, report = placement.contact_joint_placement(**args)
    assert serializable(args) == before
    assert_hint(args, proposed, report)
    return args, proposed, report


def test_sealed_failure_gets_shorter_hint_and_fresh_quantized_endpoint_checks(solved, tmp_path):
    args, proposed, report = solved
    ray = report['ray_hint']
    original = direct_args(args)
    assert any(row['collision'] for row in endpoint_checks(args, original['transforms'][1]))
    assert 0. < report['translation_distance_m'] < np.linalg.norm(original['seed'])
    assert ray['extra_proposal_margin_m'] == GRIPPER_CONTRACT_TOLERANCE_M
    assert args['contact_geometry']['gripper'].jaw_clearance_each_side_m == .002
    assert ray['minimum_target_projection_margin_m'] >= GRIPPER_CONTRACT_TOLERANCE_M
    assert ray['candidate_evaluations'] == report['evaluation_count'] == 1
    assert report['optimizer_attempts'] == []
    assert report['joint_ik_recomputed'] is False
    assert report['command_transform_is_rigid_proposal_only'] is True
    outcomes = assert_fresh_endpoint_contract(args, proposed, exact_same_branch_solution(args, proposed))
    (tmp_path/'finite_ray_endpoint_evidence.json').write_text(json.dumps(dict(
        report=report, independent_fresh_ik_endpoints=outcomes,
        complete_path_verified=False, execution_authorized=False), indent=2))


def test_original_budget_and_solver_fallback_configuration_are_preserved(solved):
    _, _, report = solved
    p = report['parameters']
    assert (p['maximum_seconds'], p['maximum_evaluations'], p['maximum_iterations'], p['maximum_solves']) == (2., 256, 48, 3)
    assert p['ray_hint']['maximum_alpha'] == 1.
    assert p['ray_hint']['maximum_intervals'] == 7
    assert p['ray_hint']['extra_proposal_margin_source'] == 'GRIPPER_CONTRACT_TOLERANCE_M'
    no_authority(report)


def test_repeat_keeps_immutable_source_and_exact_hint(solved):
    args, expected, _ = solved
    before = serializable(args)
    actual, report = placement.contact_joint_placement(**args)
    np.testing.assert_array_equal(actual, expected)
    assert serializable(args) == before
    assert_hint(args, actual, report)


@pytest.mark.parametrize('b,s,margin,wanted', [
    (0., 1., .3, [(.3, 1.)]), (1., -1., .3, [(0., .7)]),
    (.3, 0., .3, [(0., 1.)]), (.2, 0., .3, []),
    (0., 1., 2., []), (-1., -1., 0., []),
    (1., -1., 0., [(0., 1.)]), (0., 1., 0., [(0., 1.)]),
])
def test_finite_affine_inequality_boundaries(b, s, margin, wanted):
    assert placement._ray_halfspace(b, s, margin, 1.) == wanted


def test_disjunctive_separating_axes_preserve_multiple_intervals():
    # One body allows either end; another body allows the middle. A monotonic
    # bisection would discard one of these two feasible components.
    first = placement._ray_merge([(0., .3), (.7, 1.)])
    second = [(.2, .8)]
    assert placement._ray_intersect(first, second) == [(.2, .3), (.7, .8)]
    assert placement._ray_intersect(first, [(.4, .6)]) == []


def test_real_decimal_rounding_cannot_cross_declared_ray_endpoint():
    seed = np.array([.0019760917087290365, .002606385324513694, -.0023025675014756914])
    rounded = np.round(seed, decimals=12)
    assert np.linalg.norm(rounded) < .01
    assert float(rounded @ seed / (seed @ seed)) > 1.
    displacement, alpha = placement._ray_rounded_displacement(1., seed, .01, 0., 1., 1.)
    assert displacement is None and alpha is None
    displacement, alpha = placement._ray_rounded_displacement(.5, seed, .01, 0., 1., 1.)
    assert displacement is not None and 0. < alpha < 1.


def test_zero_translation_does_not_fall_through_to_deliberate_side_shift(monkeypatch):
    args = case()
    # Synthetic smaller target makes the old rejection inconsistent with the
    # supplied frozen scene. Never treat that inconsistency as a new motion.
    args['scene'] = replace(args['scene'], size=args['scene'].size*.5)
    monkeypatch.setattr('scipy.optimize.minimize', lambda *a, **k: pytest.fail('zero ray entered optimizer'))
    proposed, report = placement.contact_joint_placement(**args)
    assert proposed is None and not report['proposed'] and not report['fallback_allowed']
    assert report['ray_hint']['zero_translation_sufficient'] is True
    assert report['optimizer_attempts'] == []
    no_authority(report)


@pytest.mark.parametrize('fault', ('nan', 'outside', 'zero'))
def test_rounded_displacement_cannot_cross_finite_original_bound(fault, monkeypatch):
    args = direct_args(case())
    value = (np.full(3, np.nan) if fault == 'nan' else np.zeros(3) if fault == 'zero'
             else args['seed']/np.linalg.norm(args['seed'])*(args['bound']+1e-12))
    monkeypatch.setattr(placement.np, 'round', lambda *a, **k: value.copy())
    monkeypatch.setattr(placement, 'gripper_target_overlap', lambda *a, **k: pytest.fail('invalid displacement reached endpoint gate'))
    proposal, audit = placement._minimum_change_ray_hint(**args)
    assert proposal is None and audit['proposed'] is False


def test_independent_original_cad_kernel_can_reject_the_affine_hint(monkeypatch):
    args = direct_args(case())
    seen = []
    def collision(*a, **k):
        seen.append(True)
        return True, None
    monkeypatch.setattr(placement, 'gripper_target_overlap', collision)
    proposal, audit = placement._minimum_change_ray_hint(**args)
    assert seen and proposal is None and audit['candidate_evaluations'] == 1


@pytest.mark.parametrize('fault', ('unsupported_cad', 'changed_opening', 'changed_source'))
def test_invalid_original_binding_never_enters_ray(fault, monkeypatch):
    args = case()
    if fault == 'changed_opening': args['failure']['opening_m'] -= .001
    elif fault == 'changed_source': args['failure']['contact_terminal_failure']['source_stamp_ns'] += 1
    else:
        original = placement._failure_seed
        def unsupported(*a, **k):
            fk, command, seed = original(*a, **k)
            fk.finger_cad_model = None
            return fk, command, seed
        monkeypatch.setattr(placement, '_failure_seed', unsupported)
    monkeypatch.setattr(placement, '_minimum_change_ray_hint', lambda **k: pytest.fail('invalid original binding entered ray'))
    proposal, report = placement.contact_joint_placement(**args)
    assert proposal is None and not report['proposed']
    no_authority(report)


@pytest.mark.parametrize('exception_type', (ValueError, RuntimeError, TimeoutError))
@pytest.mark.parametrize('stage', ('first', 'last_kernel'))
def test_original_owner_exception_is_not_swallowed_by_ray(exception_type, stage, monkeypatch):
    args = case(); expected = exception_type('source owner changed')
    progress = dict(kernels=0)
    original = placement.gripper_target_overlap
    def checked(*a, **k):
        result = original(*a, **k); progress['kernels'] += 1; return result
    monkeypatch.setattr(placement, 'gripper_target_overlap', checked)
    def available():
        if stage == 'first' or progress['kernels'] >= 5: raise expected
        return True
    args['deadline_check'] = available
    with pytest.raises(exception_type) as caught: placement.contact_joint_placement(**args)
    assert caught.value is expected


def test_ray_work_and_old_optimizer_share_original_wall_deadline(monkeypatch):
    clock = dict(now=0.)
    def empty_ray(**kwargs):
        clock['now'] = 1.8
        return None, dict(candidate_evaluations=2)
    def optimizer(fun, start, **kwargs):
        clock['now'] = 2.001
        kwargs['constraints'][0]['fun'](start)
        pytest.fail('fallback reset the 2 s deadline')
    monkeypatch.setattr(placement, 'time', SimpleNamespace(monotonic=lambda: clock['now']))
    monkeypatch.setattr(placement, '_minimum_change_ray_hint', empty_ray)
    monkeypatch.setattr('scipy.optimize.minimize', optimizer)
    proposal, report = placement.contact_joint_placement(**case())
    assert proposal is None and report['elapsed_sec'] == 2.001
    assert report['reason'] == 'joint placement computation budget exhausted'
    assert report['evaluation_count'] >= 2


def test_ray_evaluations_reduce_original_optimizer_allowance(monkeypatch):
    monkeypatch.setattr(placement, '_minimum_change_ray_hint', lambda **k: (None, dict(candidate_evaluations=255)))
    with geometry_only_clock():
        proposal, report = placement.contact_joint_placement(**case())
    assert proposal is None and report['evaluation_count'] == 256
    assert 'evaluation limit' in report['reason']
    assert report['fallback_allowed']


@pytest.mark.parametrize('kind,value', [('size', .98), ('size', 1.02), ('opening', .049), ('opening', .05)])
def test_generic_changed_size_and_opening_still_use_fresh_gates(kind, value):
    args = case()
    if kind == 'size': args['scene'] = replace(args['scene'], size=args['scene'].size*value)
    else:
        args['failure']['opening_m'] = value
        args['failure']['source_sample']['accepted_gripper_opening_m'] = value
        args['contact_geometry']['opening_width_m'] = value
    proposed, report = placement.contact_joint_placement(**args)
    assert_hint(args, proposed, report)
    assert_fresh_endpoint_contract(args, proposed, exact_same_branch_solution(args, proposed))
