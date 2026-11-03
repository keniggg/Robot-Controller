"""Present CAD and future corrective paths are distinct evidence contracts.

The small G43 fixture contains exact closed-record messages and audits. It
does not contain the lost MoveIt trajectories or future real measurements.
Only the Task timing test below supplies an explicitly simulated FK cell.
"""
import base64
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
import pytest
from sensor_msgs.msg import JointState
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
from alicia_flexible_grasp.grasp import contact_execution as execution
from alicia_flexible_grasp.grasp.rich_plan_integrity import plan_id_matches_content, pose_values
from alicia_flexible_grasp.robot.pregrasp_compensation import pose_matrix
from alicia_flexible_grasp.robot.stationary_following import SerialUrdfFk, ARM_NAMES, SDK_QUANTUM_RAD as Q


FIXTURE_SHA = 'da66a49b1141e943e1b9e734621400a4130c17443c09516fabc9bc1e8a4c50f0'
ROUTE_FLAGS = (
    'unknown_approach_held_axes_feedforward_enabled',
    'unknown_contact_cartesian_compensation_enabled',
    'contact_endpoint_precision_enabled', 'measured_endpoint_check_enabled',
    'pregrasp_cartesian_compensation_enabled', 'unknown_pregrasp_feedforward_enabled',
    'unknown_pregrasp_held_axes_feedforward_enabled',
)


@pytest.fixture(autouse=True)
def no_unsimulated_boundaries(monkeypatch):
    import rospy
    import socket
    class BoundaryError(BaseException):
        pass
    def reject(*args, **kwargs):
        raise BoundaryError('GEN44 contract forbids external '+repr(args))
    for name in ('connect', 'connect_ex', 'sendto', 'bind', 'listen'):
        monkeypatch.setattr(socket.socket, name, reject)
    monkeypatch.setattr(socket, 'create_connection', reject)
    for name in ('init_node', 'Publisher', 'Subscriber', 'Service', 'ServiceProxy',
                 'wait_for_service', 'set_param', 'delete_param'):
        monkeypatch.setattr(rospy, name, reject)


def g43_case():
    path = Path(__file__).parent/'fixtures/gen44_g43_stage0_replay.json'
    raw = path.read_bytes()
    assert hashlib.sha256(raw).hexdigest() == FIXTURE_SHA
    data = json.loads(raw)
    wire = base64.b64decode(data['plan_wire']['wire_base64'])
    assert hashlib.sha256(wire).hexdigest() == data['plan_wire']['wire_sha256']
    plan = Grasp6DPlan(); plan.deserialize(wire)
    assert plan_id_matches_content(plan) and plan.plan_id == data['plan_id']
    work = Path(__file__).resolve().parents[3]
    xml = (work/data['robot_model_path']).read_text()
    assert hashlib.sha256(xml.encode()).hexdigest() == data['model_sha256']
    fk = SerialUrdfFk(xml, ARM_NAMES)
    return NS(data=data, plan=plan, fk=fk, xml=xml,
        actual=np.asarray(data['actual_reached_pregrasp']['actual_T_base_tool0']),
        opening=data['actual_reached_pregrasp']['measured_opening_width_m'])


def route():
    return dict.fromkeys(ROUTE_FLAGS, True)


def test_actual_g43_present_cad_is_safe_while_rigid_future_projection_is_not():
    c = g43_case(); original = deepcopy(c.plan)
    old = execution.plan_aperture_evaluator(c.plan, {}, c.opening, 0)(c.actual)
    assert not old['ok'] and old['code'] == 'MEASURED_APERTURE_INSUFFICIENT'
    assert old['required_open_width_m'] == pytest.approx(.052625786810793364, abs=1e-12)
    result = execution.plan_current_stage_cad_evaluator(c.plan, {}, c.opening, 0)(c.actual)
    assert result['ok'] and result['code'] == 'MEASURED_STAGE_CAD_OK'
    assert result['actual_current_minimum_support_clearance_m'] > .05
    diagnostic = result['rigid_residual_projection_diagnostic']
    assert diagnostic['code'] == old['code']
    assert diagnostic['required_open_width_m'] == old['required_open_width_m']
    assert diagnostic['diagnostic_only'] is True
    assert not result['future_path_checked'] and not result['execution_authorized']
    assert not result['physical_grip_verified'] and c.plan == original


def test_original_virtual_start_has_independent_corrective_path_evidence_not_constant_residual():
    c = g43_case(); original = c.data['original_precheck']
    assert original['success'] is True
    prediction = original['held_pregrasp_selection']['predicted_pregrasp']
    virtual = c.fk(prediction['predicted_positions_rad'])
    result = execution.plan_current_stage_cad_evaluator(c.plan, {}, c.opening, 0)(virtual)
    assert result['ok']
    projected = result['rigid_residual_projection_diagnostic']
    assert not projected['ok'] and projected['code'] == 'MEASURED_APERTURE_CAD_COLLISION'
    assert projected['projected_contact_minimum_support_clearance_m'] == pytest.approx(.0014817959331234476)
    approach = original['held_approach_prediction']
    corrected = approach['predicted_approach']
    assert corrected['proposed_delta_counts'] == [4, 0, -18, 16, 0, -32]
    assert corrected['predicted_position_error_m'] == pytest.approx(.004390718305274988)
    # Both independent trajectories were checked in the original service, but
    # their arrays were not recorded. This is archived conditional evidence,
    # never a recreated trajectory or a future hardware-success claim.
    curves = approach['guide_path_prediction']['curves']
    assert curves['predicted_actual']['geometry']['certified_minimum_clearance_m'] > .025
    assert curves['command']['geometry']['certified_minimum_clearance_m'] > .037
    contact = original['contact_prediction']
    assert contact['collision_state_checks'] == 126
    assert contact['nominal_geometry']['certified_minimum_clearance_m'] > .010
    assert contact['predicted_command_geometry']['certified_minimum_clearance_m'] > .020
    assert not contact['future_following_verified']
    assert not c.data['providers']['original_MoveIt_trajectory_recorded']


def test_recorded_producer_sources_and_signed_plan_remain_bound_without_restamping():
    c = g43_case()
    assert c.plan.header.stamp.to_nsec() == c.data['source_stamp_ns'][-1]
    for stamp in c.data['source_stamp_ns']:
        row = c.data['recorded_producer_transport_summaries'][str(stamp)]
        assert row['object']['message']['header']['stamp'] == stamp
        assert row['object']['message']['detected'] is True
        assert row['status']['value']['source_stamp_ns'] == stamp
        assert row['status']['value']['state'] == 'ready'
        assert row['mask_summary']['source_stamp_ns'] == stamp
        assert row['mask_summary']['nonzero_pixels'] > 0
        assert row['mask_summary']['mask_reconstructed_or_fabricated'] is False


@pytest.mark.parametrize('index', [0, 1])
def test_current_real_support_collision_still_rejects(index):
    c = g43_case(); below = c.actual.copy(); below[2, 3] -= .20
    result = execution.plan_current_stage_cad_evaluator(c.plan, {}, c.opening, index)(below)
    assert not result['ok'] and result['code'] == 'MEASURED_STAGE_CAD_COLLISION'
    assert 'support' in result['reason']
    assert not result['future_path_checked']


@pytest.mark.parametrize('opening', [0., .0501, float('nan')])
def test_invalid_actual_opening_never_gets_current_stage_permission(opening):
    c = g43_case()
    try:
        evaluate = execution.plan_current_stage_cad_evaluator(c.plan, {}, opening, 0)
    except ValueError:
        return  # Rejection while binding the frozen contact is also safe.
    result = evaluate(c.actual)
    assert not result['ok'] and result['code'] == 'MEASURED_STAGE_CAD_INPUT_INVALID'


def test_stage2_measured_width_remains_a_hard_close_gate():
    c = g43_case(); final = pose_matrix(pose_values(c.plan.poses[2]))
    result = execution.plan_aperture_evaluator(c.plan, {}, .040, 2)(final)
    assert not result['ok'] and result['code'] == 'MEASURED_APERTURE_INSUFFICIENT'
    assert result['projected_contact_is_measured'] is True
    assert result['required_open_width_m'] > .040
    assert 'rigid_residual_projection_diagnostic' not in result
    with pytest.raises(ValueError, match='CURRENT_STAGE_INVALID'):
        execution.plan_current_stage_cad_evaluator(c.plan, {}, c.opening, 2)


@pytest.mark.parametrize('missing', ROUTE_FLAGS)
def test_incomplete_route_keeps_historical_projection_contract(missing):
    from test_contact_execution import module as task
    cfg = route(); cfg.pop(missing)
    assert task.GraspTaskNode._contact_stage_next_path_contract(cfg, 0) is None
    assert task.GraspTaskNode._contact_stage_next_path_contract(cfg, 1) is None


@pytest.mark.parametrize('index', [0, 1])
def test_only_the_existing_independent_next_path_is_named(index):
    from test_contact_execution import module as task
    name = task.GraspTaskNode._contact_stage_next_path_contract(route(), index)
    assert name == ['fresh_held_axes_approach', 'fresh_compensated_contact_linear'][index]
    assert task.GraspTaskNode._contact_stage_next_path_contract(route(), 2) is None
    cfg = route(); cfg[ROUTE_FLAGS[0]] = 'true'
    assert task.GraspTaskNode._contact_stage_next_path_contract(cfg, index) is None


def g43_task(monkeypatch):
    from test_contact_execution import make_task, module as task
    c = g43_case(); node, _, clock, params = make_task(monkeypatch)
    params['/robot_description'] = c.xml
    params['/grasp_6d/remote/gripper_geometry'] = {}
    params['/robot/strict_execution_controller_sync_max_feedback_age_sec'] = .5
    stamp = c.data['actual_reached_pregrasp']['accepted_stamp_ns']
    clock['now'] = stamp*1e-9+.02
    node._following_epoch_ns = c.data['actual_reached_pregrasp']['epoch_ns']
    sample = JointState(); sample.header.frame_id = 'sdk_measured'
    sample.header.stamp = task.rospy.Time(stamp//10**9, stamp%10**9)
    sample.name = list(ARM_NAMES)+['right_finger']
    # A unique explicit encoder/FK-provider cell, not invented real feedback.
    key = ((np.asarray(c.data['actual_pregrasp_compensation']['measured_counts'])-2048)*Q).tolist()
    sample.position = key+[c.opening]
    node.latest_accepted_joint_state = sample
    class ExactReportedPoseProvider:
        def __init__(self, description, names):
            assert description == c.xml and tuple(names) == ARM_NAMES
            self.model_sha256 = c.fk.model_sha256
        def __call__(self, q):
            np.testing.assert_array_equal(q, key)
            return c.actual.copy()
    monkeypatch.setattr(task, 'SerialUrdfFk', ExactReportedPoseProvider)
    return c, node, clock, params, task


@pytest.mark.parametrize('complete', [False, True])
def test_real_task_selects_contract_without_granting_a_future_motion(monkeypatch, complete):
    c, node, _, _, _ = g43_task(monkeypatch)
    assert node._validate_measured_aperture(c.plan, route() if complete else {}, 0,
        'recorded pregrasp current contract') is complete
    result = node._last_measured_aperture_sample
    if complete:
        assert result['code'] == 'MEASURED_STAGE_CAD_OK'
        assert result['required_next_path_contract'] == 'fresh_held_axes_approach'
        assert result['rigid_residual_projection_diagnostic']['ok'] is False
        assert not result['future_path_checked']
    else:
        assert result['code'] == 'MEASURED_APERTURE_INSUFFICIENT'


@pytest.mark.parametrize('fault', ['stale_feedback', 'authority'])
def test_new_current_cad_contract_cannot_renew_feedback_or_authority(monkeypatch, fault):
    c, node, clock, _, _ = g43_task(monkeypatch)
    if fault == 'stale_feedback':
        clock['now'] += .6
    else:
        node._validate_bound_plan_locked = Mock(return_value=NS(ok=False, code='CONTROL_REVOKED'))
    assert not node._validate_measured_aperture(c.plan, route(), 0, 'changed real input')
    assert node._last_measured_aperture_sample['code'] == 'MEASURED_APERTURE_EVIDENCE_INVALID'
