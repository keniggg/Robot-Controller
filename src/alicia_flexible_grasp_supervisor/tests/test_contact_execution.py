"""Offline final-close replay; no ROS master, services, or publishers."""
from copy import deepcopy
import json
from pathlib import Path
import threading
from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
import pytest
from sensor_msgs.msg import JointState

import test_grasp_task_sequence as task_fixture
from alicia_flexible_grasp.grasp.contact_evidence import encode_contact_evidence
from alicia_flexible_grasp.grasp.contact_execution import plan_contact_evaluator
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id, pose_values
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SDK_QUANTUM_RAD as Q

module = task_fixture.grasp_task_node
FIXTURE = json.loads((Path(__file__).parent / 'fixtures/measured_contact_20260926.json').read_text())


def plan_from_record(name):
    row = FIXTURE['records'][name]
    plan = task_fixture.GraspTaskSequenceTest()._rich_plan(stamp_sec=10.)
    plan.model_choice = 'unknown_tabletop'
    plan.diagnostic = 'CONTACT_EXECUTION_PLAN'
    goal = plan.poses[2]
    goal.position.x, goal.position.y, goal.position.z = row['planned_position_m']
    (goal.orientation.x, goal.orientation.y, goal.orientation.z,
     goal.orientation.w) = row['planned_quaternion_xyzw']
    contact = row['frozen_contact']
    normal = plan.object_geometry.support_normal_base
    normal.x, normal.y, normal.z = contact['support_normal_base']
    plan.object_geometry.support_offset_m = contact['support_offset_m']
    plan.contact_geometry_evidence = encode_contact_evidence(
        contact_center_base=contact['contact_center_base'],
        contact_height_axis_base=contact['contact_height_axis_base'],
        contact_height_bounds_m=contact['contact_height_bounds_m'],
        minimum_contact_overlap_m=contact['minimum_contact_overlap_m'], grasp_pose=goal)
    plan.plan_id = compute_plan_id(plan)
    return plan, row


@pytest.mark.parametrize('name', ['gen11', 'gen18'])
def test_exact_plan_evidence_accepts_nominal_but_rejects_actual_empty_close(name):
    plan, row = plan_from_record(name)
    evaluate = plan_contact_evaluator(plan, {}, .04975)
    assert evaluate(row['frozen_contact']['planned_T_base_tool0'])['ok']
    result = evaluate(row['actual_T_base_tool0'])
    assert not result['ok'] and result['code'] == 'MEASURED_CONTACT_PATCH_MISS'
    assert result['actual_overlap_m'] == pytest.approx(row['expected_actual_result']['actual_overlap_m'])
    assert not result['physical_grip_verified']
    plan.contact_geometry_evidence = ''
    with pytest.raises(ValueError):
        plan_contact_evaluator(plan, {}, .04975)
    # Closure copied frozen evidence; external mutation cannot replace its band.
    assert not evaluate(row['actual_T_base_tool0'])['ok']


def make_task(monkeypatch, name='gen18', hypothetical_contact=False):
    plan, row = plan_from_record(name)
    node = module.GraspTaskNode.__new__(module.GraspTaskNode)
    node._following_evidence_lock = threading.RLock()
    node._following_epoch_ns = 1_000_000_000
    node._grasp6d_plan_guard = lambda: threading.RLock()
    node._validate_bound_plan_locked = Mock(return_value=NS(ok=True))
    node.set_state = Mock()
    state = dict(now=20.)
    monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda: module.rospy.Time.from_sec(state['now'])))
    params = {'/robot_description': FIXTURE['robot_description']}
    monkeypatch.setattr(module.rospy, 'get_param', lambda key, default=None: params.get(key, default))
    monkeypatch.setattr(module.rospy, 'loginfo', lambda *args: None)
    joints = JointState()
    joints.header.stamp = module.rospy.Time.from_sec(19.9)
    joints.header.frame_id = 'sdk_measured'
    joints.name = list(ARM_NAMES) + ['right_finger']
    # Pure offline inverse-FK + encoder quantization for a passing fixture.
    # These counts were never commanded or observed on hardware.
    counts = [816, 1583, 2881, 2020, 1237, 2047] if hypothetical_contact else row['measured_counts']
    joints.position = ((np.asarray(counts)-2048)*Q).tolist() + [.04975]
    node.latest_accepted_joint_state = joints
    node._last_fixed_endpoint_compensation_report = dict(
        code='PREGRASP_MEASURED_CONVERGED', plan_id=plan.plan_id,
        fixed_goal_pose_index=2, fixed_goal_pose=list(pose_values(plan.poses[2])),
        model_sha256=FIXTURE['model_sha256'], epoch_ns=1_000_000_000,
        accepted_stamp_ns=19_800_000_000)
    return node, plan, state, params


@pytest.mark.parametrize('name', ['gen11', 'gen18'])
def test_task_refuses_real_empty_close_despite_prior_zero_step_success(monkeypatch, name):
    node, plan, _, _ = make_task(monkeypatch, name)
    assert not node._validate_measured_contact_before_close(plan, {})
    assert node._last_measured_contact_sample['code'] == 'MEASURED_CONTACT_PATCH_MISS'
    assert 'PATCH_MISS' in node.set_state.call_args.args[1]


def test_task_accepts_only_hypothetical_encoders_reaching_contact(monkeypatch):
    node, plan, _, _ = make_task(monkeypatch, hypothetical_contact=True)
    assert node._validate_measured_contact_before_close(plan, {}), node._last_measured_contact_sample
    assert not node._last_measured_contact_sample['physical_grip_verified']


@pytest.mark.parametrize('change', ['stale', 'future', 'epoch', 'model', 'report_missing', 'missing_band', 'wrong_source', 'duplicate_joint'])
def test_task_rejects_changed_final_execution_or_feedback_evidence(monkeypatch, change):
    node, plan, state, params = make_task(monkeypatch, hypothetical_contact=True)
    if change == 'stale': state['now'] = 21.
    elif change == 'future': state['now'] = 19.
    elif change == 'epoch': node._following_epoch_ns = 19_000_000_000
    elif change == 'model': node._last_fixed_endpoint_compensation_report['model_sha256'] = '0'*64
    elif change == 'report_missing': node._last_fixed_endpoint_compensation_report = None
    elif change == 'missing_band': plan.contact_geometry_evidence = ''
    elif change == 'wrong_source': node.latest_accepted_joint_state.header.frame_id = 'heartbeat'
    elif change == 'duplicate_joint': node.latest_accepted_joint_state.name[1] = 'Joint1'
    assert not node._validate_measured_contact_before_close(plan, {})


def test_task_rechecks_age_after_geometry_and_authority_validation(monkeypatch):
    node, plan, state, _ = make_task(monkeypatch, hypothetical_contact=True)
    def slow_validation(*args):
        state['now'] = 20.5
        return NS(ok=True)
    node._validate_bound_plan_locked = slow_validation
    assert not node._validate_measured_contact_before_close(plan, {})
    assert 'EXPIRED_DURING_VALIDATION' in node._last_measured_contact_sample['reason']


@pytest.mark.parametrize('change', ['epoch', 'model', 'gripper'])
def test_task_rechecks_context_changed_during_authority_validation(monkeypatch, change):
    node, plan, _, params = make_task(monkeypatch, hypothetical_contact=True)
    def changing_validation(*args):
        if change == 'epoch':
            node._following_epoch_ns += 1
        elif change == 'model':
            params['/robot_description'] += '\n'
        else:
            params['/grasp_6d/remote/gripper_geometry'] = {'support_clearance_m': .0035}
        return NS(ok=True)
    node._validate_bound_plan_locked = changing_validation
    assert not node._validate_measured_contact_before_close(plan, {})
    expected = 'EPOCH_CHANGED' if change == 'epoch' else 'MODEL_CHANGED'
    assert expected in node._last_measured_contact_sample['reason']
