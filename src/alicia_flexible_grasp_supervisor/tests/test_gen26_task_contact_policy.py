"""Offline task commit and simulated/real close agreement for inferred boxes.

Only live parameter/time reads and target-tracking result are substituted;
the task freezes, hashes, decodes, validates and commits the real ROS plan.
"""
from copy import deepcopy
import io
import threading
from unittest.mock import Mock

import pytest

from alicia_flexible_grasp.grasp.gripper_close_contract import (
    bind_simulated_fixed_close, fixed_close_contract)
from test_contact_execution import plan_from_record
from test_gen26_contact_provenance_independent import inferred_plan
from test_grasp_task_sequence import grasp_task_node as task
from test_gripper_close_contract import settings


def frozen_task(monkeypatch, plan, live_policy=False):
    node = task.GraspTaskNode.__new__(task.GraspTaskNode)
    node.active = True
    lock = threading.RLock()
    node._grasp6d_plan_guard = lambda: lock
    node._bound_target_drift_result = Mock(return_value=task.PlanValidationResult(True))
    params = {'/grasp/unknown_require_measured_bilateral_contact': live_policy}
    monkeypatch.setattr(task.rospy, 'get_param', lambda key, default=None: params.get(key, default))
    monkeypatch.setattr(task.rospy.Time, 'now', staticmethod(lambda: plan.header.stamp))
    node._freeze_execution_plan(plan)
    return node, params


@pytest.mark.parametrize('configured,live,expected', [
    (False, False, None),
    (True, True, 'MEASURED_BILATERAL_CONTACT_REQUIRED'),
    (False, True, 'CONTACT_EVIDENCE_POLICY_CHANGED'),
    (True, False, 'CONTACT_EVIDENCE_POLICY_CHANGED'),
    ('false', False, 'CONTACT_EVIDENCE_POLICY_INVALID'),
    (False, 0, 'CONTACT_EVIDENCE_POLICY_INVALID'),
])
def test_task_commit_obeys_typed_frozen_and_live_contact_policy(monkeypatch, configured, live, expected):
    plan, _, _, _ = inferred_plan()
    node, _ = frozen_task(monkeypatch, plan, live)
    action = Mock(return_value='offline_action_result')
    result, value = node._invoke_plan_bound_action(plan,
        {'unknown_require_measured_bilateral_contact': configured}, 'contact', action)
    if expected is None:
        assert result.ok, result
        assert value == 'offline_action_result'
        action.assert_called_once_with()
    else:
        assert not result.ok and result.code == 'CONTACT_EVIDENCE_POLICY_REJECTED', result
        assert expected in result.reason
        action.assert_not_called()


def test_enabling_strict_mode_between_admission_and_action_stops_inferred_plan(monkeypatch):
    plan, _, _, _ = inferred_plan()
    node, params = frozen_task(monkeypatch, plan)
    config = {'unknown_require_measured_bilateral_contact': False}
    assert node._validate_bound_plan(plan, config).ok
    params['/grasp/unknown_require_measured_bilateral_contact'] = True
    action = Mock()
    result, _ = node._invoke_plan_bound_action(plan, config, 'close', action)
    assert not result.ok and 'POLICY_CHANGED' in result.reason
    action.assert_not_called()


def test_strict_mode_still_accepts_real_measured_contact_evidence(monkeypatch):
    plan, _ = plan_from_record('gen18')
    node, _ = frozen_task(monkeypatch, plan, True)
    action = Mock()
    result, _ = node._invoke_plan_bound_action(plan,
        {'unknown_require_measured_bilateral_contact': True}, 'contact', action)
    assert result.ok, result
    action.assert_called_once_with()


def test_inferred_preview_ros_wire_simulation_and_close_use_identical_gap():
    plan, _, contact, _ = inferred_plan()
    buffer = io.BytesIO()
    plan.serialize(buffer)
    received = type(plan)().deserialize(buffer.getvalue())
    config = settings()
    preview = bind_simulated_fixed_close(plan, {}, config)
    actual = bind_simulated_fixed_close(received, {}, config)
    assert preview == actual
    gap = preview['gripper_model']['close_target_inner_gap_m']
    assert gap == pytest.approx(contact.inferred_width_m - .002, abs=5e-9)
    node = task.GraspTaskNode.__new__(task.GraspTaskNode)
    node._command_gripper_position = Mock(return_value=True)
    node._simulated_fixed_close_contract = (plan.plan_id, fixed_close_contract(plan, config))
    assert node._close_gripper(config, object(), None, execution_plan=received)[0]
    assert node._command_gripper_position.call_args.args[1] == gap
    changed = deepcopy(config)
    changed['plan_bound_close_preload_m'] += .001
    node._command_gripper_position.reset_mock()
    assert not node._close_gripper(changed, object(), None, execution_plan=received)[0]
    node._command_gripper_position.assert_not_called()
