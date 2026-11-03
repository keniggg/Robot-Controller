"""Independent contracts for bounded joint contact-placement hints.

The two saved failures are immutable endpoint evidence. Synthetic changes below
are labelled test variations. A hint is never a signed plan, a complete path,
future hardware following, or execution authority.
"""
from copy import deepcopy
from contextlib import contextmanager, nullcontext
from dataclasses import asdict, replace
from functools import lru_cache
import ast
import hashlib
import json
from pathlib import Path
import time
from types import SimpleNamespace
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

import alicia_flexible_grasp.grasp.contact_joint_placement as placement
from alicia_flexible_grasp.grasp.contact_height_margin import evaluate_contact_height_margin
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry, _box_corners, _stage_boxes
from alicia_flexible_grasp.robot.endpoint_correction import sdk_counts
from alicia_flexible_grasp.robot.observation_path_guard import (
    FrozenObservationScene, ObservationPathError, SerialUrdfFk,
    gripper_target_overlap, validate_observation_trajectory,
)
from alicia_flexible_grasp.robot.pregrasp_compensation import pose_matrix
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SDK_QUANTUM_RAD
from alicia_flexible_grasp.vision.remote_grasp6d_client import CandidateContractError


FIXTURE = Path(__file__).parent / 'fixtures/contact_joint_terminal_20261027.json'
ROTATION = getattr(Rotation, 'from_matrix', None) or Rotation.from_dcm


def rotation_matrix(value):
    return (getattr(value, 'as_matrix', None) or value.as_dcm)()


@lru_cache(maxsize=1)
def saved():
    value = json.loads(FIXTURE.read_text())
    assert value['schema_version'] == 1
    assert hashlib.sha256(value['model_xml'].encode()).hexdigest() == value['model_sha256']
    assert len(value['rows']) == 2
    return value


def case(index=0):
    return fixture_case(saved(), index)


def fixture_case(data, index=0):
    row = deepcopy(data['rows'][index])
    scene = FrozenObservationScene(**{
        key: np.asarray(value, dtype=float) if key in ('normal', 'center', 'rotation', 'size') else value
        for key, value in row['scene'].items()
    })
    gripper = GripperGeometry(**row['gripper'])
    evidence = row['contact_geometry_evidence']
    geometry = dict(
        gripper=gripper, planned_T_base_tool0=pose_matrix(row['goal_pose']),
        contact_center_base=np.asarray(evidence['contact_center_base']),
        contact_height_axis_base=np.asarray(evidence['contact_height_axis_base']),
        contact_height_bounds_m=tuple(evidence['contact_height_bounds_m']),
        minimum_contact_overlap_m=evidence['minimum_contact_overlap_m'],
        support_normal_base=scene.normal.copy(), support_offset_m=scene.offset,
        opening_width_m=min(gripper.max_inner_gap_m, row['planned_opening_m']),
    )
    return dict(
        failure=row['failure'], plan_id=row['failure']['plan_id'],
        source_stamp_ns=scene.source_ns, goal_pose=row['goal_pose'],
        support_normal_base=scene.normal.copy(), model_xml=data['model_xml'],
        maximum_translation_m=row['maximum_translation_m'], scene=scene,
        contact_geometry=geometry, translation_evidence_limit_m=row['translation_evidence_limit_m'],
        deadline_check=lambda: True,
    )


@contextmanager
def geometry_only_clock():
    """Isolate finite geometry work from wall time; never certify a 2 s solve."""
    evidence = dict(clock_mode='deterministic_geometry_only',
        production_time_budget_seconds=placement.MAXIMUM_SECONDS,
        production_budget_or_performance_pass_claimed=False)
    started = time.monotonic()
    try:
        # Replace this module reference only, for this call's duration. The
        # stdlib clock, caller checks and downstream path clock remain real.
        with patch.object(placement, 'time', SimpleNamespace(monotonic=lambda: 0.)):
            yield evidence
    finally:
        evidence['actual_wall_sec'] = time.monotonic()-started


def geometry_proposal(args):
    with geometry_only_clock() as clock_evidence:
        proposed, report = placement.contact_joint_placement(**args)
    report['test_clock_evidence'] = clock_evidence
    return proposed, report


def serializable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (FrozenObservationScene, GripperGeometry)):
        return {key: serializable(item) for key, item in asdict(value).items()}
    if isinstance(value, dict):
        return {key: serializable(item) for key, item in value.items() if not callable(item)}
    if isinstance(value, (list, tuple)):
        return [serializable(item) for item in value]
    return value


def no_authority(audit):
    for key in ('execution_authorized', 'motion_authorized', 'path_verified',
                'future_following_verified', 'target_relocated', 'joint_ik_recomputed'):
        assert audit[key] is False, (key, audit)
    assert audit['command_transform_is_rigid_proposal_only'] is True


def bias_and_nominal(args):
    sample = args['failure']['source_sample']
    bias = np.asarray(sample['sdk_positions_rad']) - sample['accepted_positions_rad']
    command = np.asarray(args['failure']['contact_terminal_failure']['command_endpoint_rad'])
    return bias, command - bias


def endpoint_checks(args, transform):
    fk = SerialUrdfFk(args['model_xml'], ARM_NAMES)
    geometry, scene = args['contact_geometry'], args['scene']
    gripper, opening = geometry['gripper'], args['failure']['opening_m']
    result = []
    for body, center, size in _stage_boxes(transform, gripper, opening, 'y', 'z'):
        collision, proof = gripper_target_overlap(
            fk, gripper, opening, body, transform, center, size, scene)
        clearance = float(np.min(_box_corners(center, transform[:3, :3], size)
                                 @ scene.normal + scene.offset))
        result.append(dict(body=body, collision=bool(collision), clearance=clearance, proof=proof))
    return result


def assert_endpoint_clear(args, transform):
    checks = endpoint_checks(args, transform)
    assert all(not item['collision'] for item in checks), checks
    assert min(item['clearance'] for item in checks) >= args['contact_geometry']['gripper'].support_clearance_m


def assert_hint(args, proposed, report):
    assert proposed is not None, report
    assert report['proposed'] and report['fallback_allowed'] and report['input_valid']
    no_authority(report)
    original = pose_matrix(args['goal_pose'])
    np.testing.assert_array_equal(proposed[:3, :3], original[:3, :3])
    delta = proposed[:3, 3] - original[:3, 3]
    assert 0 < np.linalg.norm(delta) <= args['maximum_translation_m']
    np.testing.assert_allclose(delta, report['translation_base_m'], atol=1e-16, rtol=0)
    assert report['plan_id'] == args['plan_id']
    assert report['source_stamp_ns'] == args['source_stamp_ns']
    assert report['epoch_ns'] == args['failure']['epoch_ns']
    assert report['scene_digest'] == args['scene'].digest
    assert report['geometry_digest'] == args['scene'].geometry_digest
    assert 0 < report['evaluation_count'] <= placement.MAXIMUM_EVALUATIONS
    assert report['elapsed_sec'] <= placement.MAXIMUM_SECONDS
    fk = SerialUrdfFk(args['model_xml'], ARM_NAMES)
    command = fk(args['failure']['contact_terminal_failure']['command_endpoint_rad'])
    command[:3, 3] += delta
    assert_endpoint_clear(args, proposed)
    assert_endpoint_clear(args, command)
    geometry = dict(args['contact_geometry'], planned_T_base_tool0=proposed)
    height = evaluate_contact_height_margin(
        translation_evidence_limit_m=args['translation_evidence_limit_m'],
        actual_T_base_tool0=proposed, **geometry)
    assert height['ok'], height


def exact_same_branch_solution(args, proposed):
    fk = SerialUrdfFk(args['model_xml'], ARM_NAMES)
    bias, initial = bias_and_nominal(args)
    lower = np.maximum(fk.limits[:, 0], fk.limits[:, 0] - bias) + 1e-12
    upper = np.minimum(fk.limits[:, 1], fk.limits[:, 1] - bias) - 1e-12
    def residual(q):
        actual = fk(q)
        return np.r_[1000 * (actual[:3, 3] - proposed[:3, 3]),
                     100 * ROTATION(proposed[:3, :3].T @ actual[:3, :3]).as_rotvec()]
    solved = least_squares(residual, np.clip(initial, lower, upper), bounds=(lower, upper),
                           max_nfev=80, ftol=1e-12, xtol=1e-12, gtol=1e-12)
    assert np.linalg.norm(residual(solved.x)) < 1e-6, solved.message
    return solved.x


def fresh_endpoint_contract(args, proposed, joints):
    fk = SerialUrdfFk(args['model_xml'], ARM_NAMES)
    bias, _ = bias_and_nominal(args)
    quantized = (sdk_counts(joints + bias) - 2048) * SDK_QUANTUM_RAD
    outcomes = {}
    for label, q in (('nominal', joints), ('command', joints + bias),
                     ('quantized_command', quantized), ('quantized_actual', quantized - bias)):
        actual = fk(q)
        outcomes[label] = dict(joints=q.tolist(), endpoints=endpoint_checks(args, actual))
        if label in ('nominal', 'quantized_actual'):
            height = evaluate_contact_height_margin(
                translation_evidence_limit_m=args['translation_evidence_limit_m'],
                actual_T_base_tool0=actual,
                **dict(args['contact_geometry'], planned_T_base_tool0=proposed))
            outcomes[label]['height'] = height
    return outcomes


def fresh_contract_passes(args, outcomes):
    clearance = args['contact_geometry']['gripper'].support_clearance_m
    return all(all(not item['collision'] and item['clearance'] >= clearance
                   for item in row['endpoints'])
               and row.get('height', {'ok': True})['ok']
               for row in outcomes.values())


def assert_fresh_endpoint_contract(args, proposed, joints):
    outcomes = fresh_endpoint_contract(args, proposed, joints)
    assert fresh_contract_passes(args, outcomes), outcomes
    return outcomes


@pytest.fixture(scope='module', params=(0, 1), ids=('record_family_a', 'record_family_b'))
def solved_case(request):
    args = case(request.param)
    before = serializable(args)
    proposed, report = geometry_proposal(args)
    assert serializable(args) == before
    assert_hint(args, proposed, report)
    joints = exact_same_branch_solution(args, proposed)
    return args, proposed, report, joints


def test_actual_failures_have_joint_hint_and_fresh_quantized_endpoints(solved_case, tmp_path):
    args, proposed, report, joints = solved_case
    fk = SerialUrdfFk(args['model_xml'], ARM_NAMES)
    bias, original = bias_and_nominal(args)
    assert any(item['collision'] for item in endpoint_checks(args, fk(original + bias)))
    assert_endpoint_clear(args, fk(original))
    outcomes = assert_fresh_endpoint_contract(args, proposed, joints)
    (tmp_path / 'endpoint_only_evidence.json').write_text(json.dumps(dict(
        proposal=report, endpoints=outcomes, fresh_same_branch_ik=True,
        historical_moveit_path_replayed=False, complete_curve_certificate=False,
        whole_arm_checked=False, execution_authorized=False), indent=2))


def test_repeat_is_bitwise_deterministic_without_mutating_input(solved_case, tmp_path):
    args, expected, _, _ = solved_case
    before = serializable(args)
    proposed, report = geometry_proposal(args)
    (tmp_path/'repeat_geometry_contract.json').write_text(json.dumps(report, indent=2))
    np.testing.assert_array_equal(proposed, expected)
    assert serializable(args) == before
    assert_hint(args, proposed, report)


@pytest.mark.parametrize('fault', [
    'success', 'scope', 'authorized', 'stage', 'code', 'reason', 'no_terminal',
    'schema_bool', 'terminal_plan', 'terminal_source', 'terminal_curve',
    'terminal_stage', 'not_endpoint', 'terminal_authorized', 'model', 'epoch',
    'sample_epoch_float', 'sample_epoch_bool', 'sample_joint_names',
    'sample_kind', 'sample_authority', 'sdk_stale', 'feedback_future',
    'sdk_stamp_float', 'feedback_stamp_bool', 'opening_stamp', 'opening_mismatch',
    'mixed_bool_sdk', 'mixed_bool_feedback', 'mixed_bool_endpoint',
    'nan_endpoint', 'short_endpoint', 'large_bias', 'goal_changed',
    'goal_mixed_bool', 'goal_nonunit', 'scene_source', 'scene_plan',
    'scene_no_digest', 'scene_no_geometry', 'scene_no_track',
    'scene_bool_size', 'scene_zero_size', 'scene_nan_center',
    'scene_reflection', 'scene_nonunit', 'normal_mismatch',
    'geometry_plane_mismatch', 'geometry_goal_mismatch', 'geometry_missing',
    'no_following', 'source_bool',
])
def test_invalid_or_unbound_input_never_reaches_optimizer(fault, monkeypatch):
    args = case()
    f = args['failure']; t = f['contact_terminal_failure']; s = f['source_sample']
    if fault == 'success': f['success'] = True
    elif fault == 'scope': f['scope'] = 'unbound'
    elif fault == 'authorized': f['execution_authorized'] = True
    elif fault == 'stage': f['failed_stage'] = 'approach'
    elif fault == 'code': f['failure_code'] = 'RAW_INPUT_INVALID'
    elif fault == 'reason': f['reason'] = 'unrelated rejection'
    elif fault == 'no_terminal': f.pop('contact_terminal_failure')
    elif fault == 'schema_bool': t['schema_version'] = True
    elif fault == 'terminal_plan': t['plan_id'] = 'other'
    elif fault == 'terminal_source': t['source_stamp_ns'] += 1
    elif fault == 'terminal_curve': t['curve'] = 'nominal'
    elif fault == 'terminal_stage': t['stage'] = 'pregrasp'
    elif fault == 'not_endpoint': t['endpoint_screen_only'] = False
    elif fault == 'terminal_authorized': t['execution_authorized'] = True
    elif fault == 'model': s['model_sha256'] = 'other'
    elif fault == 'epoch': s['epoch_ns'] += 1
    elif fault == 'sample_epoch_float': s['epoch_ns'] = float(s['epoch_ns'])
    elif fault == 'sample_epoch_bool': s['epoch_ns'] = True
    elif fault == 'sample_joint_names': s['joint_names'] = list(reversed(ARM_NAMES))
    elif fault == 'sample_kind': s['measurement_kind'] = 'simulated'
    elif fault == 'sample_authority': s['certifies_command_response_or_calibration'] = True
    elif fault == 'sdk_stale': s['sdk_stamp_ns'] -= 1000000000
    elif fault == 'feedback_future': s['accepted_stamp_ns'] += 1000000000
    elif fault == 'sdk_stamp_float': s['sdk_stamp_ns'] = float(s['sdk_stamp_ns'])
    elif fault == 'feedback_stamp_bool': s['accepted_stamp_ns'] = True
    elif fault == 'opening_stamp': s['opening_stamp_ns'] += 1
    elif fault == 'opening_mismatch': f['opening_m'] += .001
    elif fault == 'mixed_bool_sdk': s['sdk_positions_rad'][0] = True
    elif fault == 'mixed_bool_feedback': s['accepted_positions_rad'][0] = True
    elif fault == 'mixed_bool_endpoint': t['command_endpoint_rad'][0] = True
    elif fault == 'nan_endpoint': t['command_endpoint_rad'][0] = float('nan')
    elif fault == 'short_endpoint': t['command_endpoint_rad'].pop()
    elif fault == 'large_bias': s['sdk_positions_rad'][0] = s['accepted_positions_rad'][0] + .035001
    elif fault == 'goal_changed': args['goal_pose'][0] += .001
    elif fault == 'goal_mixed_bool': args['goal_pose'][0] = True
    elif fault == 'goal_nonunit': args['goal_pose'][3:] = [0., 0., 0., 2.]
    elif fault == 'scene_source': args['scene'] = replace(args['scene'], source_ns=args['source_stamp_ns'] + 1)
    elif fault == 'scene_plan': args['scene'] = replace(args['scene'], plan_id='other')
    elif fault == 'scene_no_digest': args['scene'] = replace(args['scene'], digest='')
    elif fault == 'scene_no_geometry': args['scene'] = replace(args['scene'], geometry_digest='')
    elif fault == 'scene_no_track': args['scene'] = replace(args['scene'], track_id='')
    elif fault == 'scene_bool_size': args['scene'] = replace(args['scene'], size=[True, .02, .03])
    elif fault == 'scene_zero_size': args['scene'] = replace(args['scene'], size=np.array([0., .02, .03]))
    elif fault == 'scene_nan_center': args['scene'] = replace(args['scene'], center=np.full(3, np.nan))
    elif fault == 'scene_reflection': args['scene'] = replace(args['scene'], rotation=np.diag([1., 1., -1.]))
    elif fault == 'scene_nonunit': args['scene'] = replace(args['scene'], normal=args['scene'].normal * 2.)
    elif fault == 'normal_mismatch': args['support_normal_base'] = [0., 0., 1.]
    elif fault == 'geometry_plane_mismatch': args['contact_geometry']['support_offset_m'] += .001
    elif fault == 'geometry_goal_mismatch': args['contact_geometry']['planned_T_base_tool0'][0, 3] += .001
    elif fault == 'geometry_missing': args['contact_geometry'] = None
    elif fault == 'no_following': f.pop('source_sample')
    elif fault == 'source_bool': args['source_stamp_ns'] = True
    def forbidden(*unused, **kwargs):
        pytest.fail('invalid source entered numerical optimizer')
    monkeypatch.setattr('scipy.optimize.minimize', forbidden)
    proposed, report = placement.contact_joint_placement(**args)
    assert proposed is None and not report['proposed'] and not report['fallback_allowed'], report
    assert report['evaluation_count'] == 0
    no_authority(report)


@pytest.mark.parametrize('field', ('translation_evidence_limit_m', 'maximum_translation_m'))
@pytest.mark.parametrize('value', (None, True, False, 0., -.001, float('nan'), float('inf')))
def test_raw_invalid_numeric_gate_never_generates_a_candidate(field, value, monkeypatch):
    args = case(); args[field] = value
    monkeypatch.setattr('scipy.optimize.minimize', lambda *a, **k: pytest.fail('invalid raw gate optimized'))
    proposed, report = placement.contact_joint_placement(**args)
    assert proposed is None and report['evaluation_count'] == 0 and not report['fallback_allowed']
    no_authority(report)


@pytest.mark.parametrize('opening', (0., -.001, .055, True, float('nan'), float('inf')))
def test_invalid_or_unsupported_opening_is_not_an_optimization_input(opening, monkeypatch):
    args = case()
    args['failure']['opening_m'] = opening
    args['failure']['source_sample']['accepted_gripper_opening_m'] = opening
    args['contact_geometry']['opening_width_m'] = opening
    monkeypatch.setattr('scipy.optimize.minimize', lambda *a, **k: pytest.fail('invalid opening optimized'))
    proposed, report = placement.contact_joint_placement(**args)
    assert proposed is None and report['evaluation_count'] == 0 and not report['fallback_allowed']
    no_authority(report)


@pytest.mark.parametrize('kind', ('zero_bias', 'seed_outside_remaining'))
def test_valid_input_without_midpoint_keeps_original_geometric_gates(kind, monkeypatch, tmp_path):
    args = case(); bias, nominal = bias_and_nominal(args)
    if kind == 'zero_bias':
        args['failure']['source_sample']['sdk_positions_rad'] = list(
            args['failure']['source_sample']['accepted_positions_rad'])
        args['failure']['contact_terminal_failure']['command_endpoint_rad'] = nominal.tolist()
    else:
        args['maximum_translation_m'] = 1e-7
    # This case verifies the geometry/seed contract, not host scheduling. A
    # busy full-regression host may correctly consume the real two-second
    # limit before the first solve completes. Replace only this module's
    # clock reference; do not patch the shared stdlib/pytest clock. Separate
    # real-clock budget-expiry and owner-cancellation tests remain unchanged.
    monkeypatch.setattr(placement, 'time', SimpleNamespace(monotonic=lambda: 0.))
    started = time.monotonic()
    proposed, report = placement.contact_joint_placement(**args)
    (tmp_path/'midpoint_geometry_contract.json').write_text(json.dumps(dict(
        kind=kind, clock_mode='deterministic_geometry_only',
        actual_wall_sec=time.monotonic()-started, report=report,
        production_time_budget_seconds=placement.MAXIMUM_SECONDS,
        production_budget_or_performance_pass_claimed=False), indent=2))
    assert report['fallback_allowed'] is True and report['input_valid'] is True
    assert report['seed_available'] is False
    assert report['optimizer_attempts'], report
    assert report['optimizer_attempts'][0]['seed']['policy'] == 'original_nominal_origin'
    if proposed is not None:
        assert_hint(args, proposed, report)
    else:
        assert not report['proposed']
    no_authority(report)


def replace_model(args, xml):
    digest = hashlib.sha256(xml.encode()).hexdigest()
    args['model_xml'] = xml
    f = args['failure']
    for field in (f, f['source_sample'], f['contact_terminal_failure']):
        field['model_sha256'] = digest


def test_unvalidated_finger_model_has_no_joint_hint(monkeypatch):
    args = case()
    tree = ET.fromstring(args['model_xml'])
    finger = tree.find("./joint[@name='left_finger']/origin")
    xyz = np.fromstring(finger.attrib['xyz'], sep=' '); xyz[0] += .0001
    finger.attrib['xyz'] = ' '.join(map(str, xyz))
    replace_model(args, ET.tostring(tree, encoding='unicode'))
    assert SerialUrdfFk(args['model_xml'], ARM_NAMES).finger_cad_model is None
    monkeypatch.setattr('scipy.optimize.minimize', lambda *a, **k: pytest.fail('unsupported CAD optimized'))
    proposed, report = placement.contact_joint_placement(**args)
    assert proposed is None and report['evaluation_count'] == 0
    no_authority(report)
    # The earlier original conservative retry can remain available. No model
    # substitution or new CAD separation proof is created here.
    assert report['fallback_allowed'] is True


@lru_cache(maxsize=None)
def varied_proposal(variation):
    args = case()
    if variation == 'smaller_object':
        args['scene'] = replace(args['scene'], size=args['scene'].size * np.array([.9, .85, .9]))
    elif variation == 'shorter_object':
        args['scene'] = replace(args['scene'], size=args['scene'].size * np.array([.9, 1., 1.]))
    else:
        opening = dict(wider_opening=.05, narrower_opening=.049,
                       opening_0475=.0475, opening_0485=.0485,
                       opening_0495=.0495, opening_0499=.0499)[variation]
        args['failure']['opening_m'] = opening
        args['failure']['source_sample']['accepted_gripper_opening_m'] = opening
        args['contact_geometry']['opening_width_m'] = opening
    # These synthetic frozen inputs do not claim freshly observed bilateral
    # evidence. That separate caller gate remains mandatory before any plan.
    proposed, report = geometry_proposal(args)
    return args, proposed, report


SIZE_VARIATIONS = ('smaller_object', 'shorter_object')
OPENING_VARIATIONS = ('wider_opening', 'narrower_opening', 'opening_0475',
                      'opening_0485', 'opening_0495', 'opening_0499')


@lru_cache(maxsize=None)
def varied_fresh_outcome(variation):
    args, proposed, report = varied_proposal(variation)
    if proposed is None:
        return None
    assert_hint(args, proposed, report)
    joints = exact_same_branch_solution(args, proposed)
    return fresh_endpoint_contract(args, proposed, joints)


@pytest.mark.parametrize('variation', SIZE_VARIATIONS + OPENING_VARIATIONS)
def test_varied_object_sizes_and_openings_are_rechecked(variation, tmp_path):
    args, proposed, report = varied_proposal(variation)
    (tmp_path/'bounded_proposal.json').write_text(json.dumps(report, indent=2))
    if proposed is None:
        assert report['input_valid'] and report['fallback_allowed'] and not report['proposed'], report
        no_authority(report)
        return
    outcomes = varied_fresh_outcome(variation)
    (tmp_path/'fresh_endpoint_contract.json').write_text(json.dumps(outcomes, indent=2))
    (tmp_path/'classification.json').write_text(json.dumps(dict(
        hint_found=True, original_fresh_endpoint_contract_passed=fresh_contract_passes(args, outcomes),
        execution_authorized=False, complete_curve_certificate=False,
        downstream_rejection_required=not fresh_contract_passes(args, outcomes)), indent=2))


def test_generic_positive_matrix_is_nonempty_for_size_and_opening():
    # Optional conservative fallbacks above are not called found solutions.
    # Distinct size and opening positives must also exist, so an implementation
    # returning None for every new geometry cannot pass this suite.
    for variations in (SIZE_VARIATIONS, OPENING_VARIATIONS):
        assert any(varied_fresh_outcome(name) is not None and fresh_contract_passes(
            varied_proposal(name)[0], varied_fresh_outcome(name)) for name in variations), variations


def test_changed_opening_hint_does_not_waive_quantized_actual_height_rejection():
    args, proposed, report = varied_proposal('narrower_opening')
    assert proposed is not None
    outcomes = varied_fresh_outcome('narrower_opening')
    assert all(not item['collision'] for row in outcomes.values() for item in row['endpoints'])
    assert outcomes['nominal']['height']['ok']
    assert not outcomes['quantized_actual']['height']['ok']
    assert not fresh_contract_passes(args, outcomes)
    no_authority(report)


FRAME_VARIATIONS = (
    ((0., 0., 1.1), (.12, -.07, .04)),
    ((.23, -.31, -.47), (-.08, .04, .09)),
)


@lru_cache(maxsize=None)
def frame_proposal(angles, translation):
    args = case()
    rotation = rotation_matrix(Rotation.from_euler('xyz', angles))
    translation = np.asarray(translation)
    original = pose_matrix(args['goal_pose'])
    tree = ET.fromstring(args['model_xml'])
    origin = tree.find("./joint[@name='Joint1']/origin")
    before_r = rotation_matrix(Rotation.from_euler('xyz', np.fromstring(origin.attrib['rpy'], sep=' ')))
    before_t = np.fromstring(origin.attrib['xyz'], sep=' ')
    origin.attrib['rpy'] = ' '.join(map(str, ROTATION(rotation @ before_r).as_euler('xyz')))
    origin.attrib['xyz'] = ' '.join(map(str, rotation @ before_t + translation))
    replace_model(args, ET.tostring(tree, encoding='unicode'))
    scene = args['scene']; normal = rotation @ scene.normal
    args['scene'] = replace(scene, normal=normal, offset=scene.offset - normal @ translation,
        center=rotation @ scene.center + translation, rotation=rotation @ scene.rotation)
    args['support_normal_base'] = normal.copy()
    target = original.copy(); target[:3, :3] = rotation @ original[:3, :3]
    target[:3, 3] = rotation @ original[:3, 3] + translation
    args['goal_pose'] = np.r_[target[:3, 3], ROTATION(target[:3, :3]).as_quat()].tolist()
    geometry = args['contact_geometry']
    geometry.update(planned_T_base_tool0=target,
        contact_center_base=rotation @ geometry['contact_center_base'] + translation,
        contact_height_axis_base=rotation @ geometry['contact_height_axis_base'],
        support_normal_base=normal.copy(), support_offset_m=args['scene'].offset)
    fk = SerialUrdfFk(args['model_xml'], ARM_NAMES)
    assert fk.finger_cad_model is not None
    _, nominal = bias_and_nominal(args)
    np.testing.assert_allclose(fk(nominal)[:3, 3], target[:3, 3], atol=.0005, rtol=0)
    proposed, report = geometry_proposal(args)
    return args, proposed, report


@pytest.mark.parametrize('angles,translation', FRAME_VARIATIONS)
def test_rigid_frame_and_support_direction_changes_preserve_geometry(angles, translation, tmp_path):
    args, proposed, report = frame_proposal(angles, translation)
    (tmp_path/'bounded_proposal.json').write_text(json.dumps(report, indent=2))
    if proposed is None:
        assert report['input_valid'] and report['fallback_allowed'] and not report['proposed'], report
        no_authority(report)
        return
    assert_hint(args, proposed, report)
    joints = exact_same_branch_solution(args, proposed)
    outcomes = assert_fresh_endpoint_contract(args, proposed, joints)
    (tmp_path/'fresh_endpoint_contract.json').write_text(json.dumps(outcomes, indent=2))


def test_generic_positive_matrix_includes_changed_pose_and_support_direction():
    assert any(frame_proposal(*variation)[1] is not None for variation in FRAME_VARIATIONS)


@pytest.mark.parametrize('kind', ('oversized_target', 'narrow_opening', 'invalid_height_interval'))
def test_real_collision_or_missing_height_cannot_become_a_joint_accept(kind):
    args = case()
    if kind == 'oversized_target':
        args['scene'] = replace(args['scene'], size=args['scene'].size * 3.)
    elif kind == 'narrow_opening':
        args['failure']['opening_m'] = .02
        args['failure']['source_sample']['accepted_gripper_opening_m'] = .02
        args['contact_geometry']['opening_width_m'] = .02
    else:
        args['contact_geometry']['contact_height_bounds_m'] = (1., 1.001)
    fk = SerialUrdfFk(args['model_xml'], ARM_NAMES)
    if kind != 'invalid_height_interval':
        assert any(item['collision'] for item in endpoint_checks(args, pose_matrix(args['goal_pose'])))
        # Prove actual unpadded mesh overlap, not just failure to find a SAT
        # separating axis. The original target and original CAD are retained.
        penetrates = False
        transform = pose_matrix(args['goal_pose'])
        for body in ('left_finger', 'right_finger'):
            for part in fk.finger_cad_model._body_components(body):
                vertices = part.vertices_at_zero + args['failure']['opening_m'] * part.velocity_per_opening
                world = vertices @ transform[:3, :3].T + transform[:3, 3]
                local = (world - args['scene'].center) @ args['scene'].rotation
                penetrates |= bool(np.any(np.all(args['scene'].size / 2 - np.abs(local) > 1e-6, axis=1)))
        assert penetrates
    proposed, report = placement.contact_joint_placement(**args)
    assert proposed is None and not report['proposed'], report
    no_authority(report)


def stream_cancel_type():
    # Load only the actual exception class, with no remote-node imports or
    # service initialization. The class source is part of the test's inputs.
    import alicia_flexible_grasp
    package = Path(alicia_flexible_grasp.__file__).resolve().parents[2]
    source = package / 'scripts/remote_grasp6d_node.py'
    tree = ast.parse(source.read_text(), filename=str(source))
    definition = next(node for node in tree.body
                      if isinstance(node, ast.ClassDef) and node.name == 'StreamResultCancelled')
    namespace = {}
    exec(compile(ast.Module(body=[definition], type_ignores=[]), str(source), 'exec'), namespace)
    return namespace['StreamResultCancelled']


@pytest.mark.parametrize('exception_type', (
    RuntimeError, ValueError, TimeoutError, CandidateContractError, stream_cancel_type(),
))
@pytest.mark.parametrize('stage', ('initial', 'computing', 'last_endpoint'))
def test_original_owner_deadline_exception_object_propagates(exception_type, stage, monkeypatch, tmp_path):
    args = case()
    expected = (exception_type('GENERATION_STALE', 'independent owner/deadline cancellation')
                if exception_type is CandidateContractError
                else exception_type('independent owner/deadline cancellation'))
    progress = dict(ticks=0, checked=0)
    original_check = placement.gripper_target_overlap
    def checked(*a, **k):
        result = original_check(*a, **k)
        progress['checked'] += 1
        return result
    monkeypatch.setattr(placement, 'gripper_target_overlap', checked)
    def available():
        progress['ticks'] += 1
        stop = (stage == 'initial' or
                stage == 'computing' and progress['ticks'] >= 6 or
                stage == 'last_endpoint' and progress['checked'] == 5)
        if stop:
            raise expected
        return True
    args['deadline_check'] = available
    clock_context = geometry_only_clock() if stage == 'last_endpoint' else nullcontext()
    with clock_context as clock_evidence:
        with pytest.raises(exception_type) as caught:
            placement.contact_joint_placement(**args)
    if clock_evidence is not None:
        (tmp_path/'last_endpoint_cancellation_clock.json').write_text(
            json.dumps(clock_evidence, indent=2))
    assert caught.value is expected
    if stage == 'last_endpoint':
        assert progress['checked'] == 5


@pytest.mark.parametrize('value', (False, None, 0, 1, np.bool_(True)))
@pytest.mark.parametrize('stage', ('initial', 'computing'))
def test_non_true_owner_deadline_check_stops_without_fallback(value, stage):
    args = case()
    calls = []
    def available():
        calls.append(time.monotonic())
        return value if stage == 'initial' or len(calls) >= 6 else True
    args['deadline_check'] = available
    proposed, report = placement.contact_joint_placement(**args)
    assert proposed is None and not report['fallback_allowed']
    no_authority(report)


def test_actual_budget_expiry_does_not_create_a_hint(monkeypatch):
    args = case()
    # Reduce only this test's local computation allowance. The clock remains
    # actual monotonic time, and no deadline is renewed by a solver callback.
    monkeypatch.setattr(placement, 'MAXIMUM_SECONDS', 1e-9)
    proposed, report = placement.contact_joint_placement(**args)
    assert proposed is None and not report['fallback_allowed']
    assert 'budget' in report['reason']
    no_authority(report)


def test_valid_input_local_evaluation_budget_preserves_conservative_fallback(monkeypatch):
    args = case()
    monkeypatch.setattr(placement, 'MAXIMUM_EVALUATIONS', 1)
    proposed, report = placement.contact_joint_placement(**args)
    assert proposed is None and report['input_valid'] and report['fallback_allowed']
    assert report['evaluation_count'] == 1 and 'evaluation limit' in report['reason']
    no_authority(report)


@pytest.mark.parametrize('curve', ('nominal', 'predicted_command'))
def test_clear_endpoints_do_not_authorize_an_interior_collision(solved_case, curve):
    import rospy
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    args, _, report, nominal = solved_case
    bias, old_nominal = bias_and_nominal(args)
    start = nominal if curve == 'nominal' else nominal + bias
    middle = old_nominal + bias
    fk = SerialUrdfFk(args['model_xml'], ARM_NAMES)
    assert_endpoint_clear(args, fk(start))
    assert any(item['collision'] for item in endpoint_checks(args, fk(middle)))
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = list(ARM_NAMES)
    for i, q in enumerate((start, middle, start)):
        trajectory.joint_trajectory.points.append(JointTrajectoryPoint(
            positions=q.tolist(), velocities=[0.] * 6, accelerations=[0.] * 6,
            time_from_start=rospy.Duration.from_sec(float(i))))
    # Deliberately constructed detour: the optimizer's endpoint remains clear,
    # yet the unchanged full-curve guard must reject this path. Real clock and
    # original 10 s / 8192 check limit; no MoveIt or hardware service is used.
    with pytest.raises(ObservationPathError, match='collision'):
        validate_observation_trajectory(trajectory, args['scene'], fk,
            args['contact_geometry']['gripper'], args['failure']['opening_m'],
            deepcopy(trajectory.joint_trajectory.points[0]), max_seconds=10., max_checks=8192)
    no_authority(report)

