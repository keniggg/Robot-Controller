"""Unknown approach uses one bounded episode and retains actual arrival gates."""
from copy import deepcopy
import hashlib
import threading
from types import SimpleNamespace as NS
from unittest import mock

import pytest

from alicia_flexible_grasp.grasp.rich_plan_integrity import pose_values
from test_pregrasp_task_compensation import task_case, task


@pytest.fixture(autouse=True)
def report_clock_and_model(monkeypatch):
    monkeypatch.setattr(task.rospy, 'get_param', lambda key, default=None:
                        'test robot description' if key == '/robot_description' else default)
    monkeypatch.setattr(task.rospy.Time, 'now', lambda: task.rospy.Time.from_sec(20.))


def case():
    node, plan, pose, cfg, old_move = task_case()
    plan.model_choice = 'unknown_tabletop'
    plan.header.stamp = task.rospy.Time.from_sec(10.)
    node._following_evidence_lock = threading.RLock()
    node._following_epoch_ns = 1_000_000_000
    pose.pose = deepcopy(plan.poses[1])
    cfg.update(unknown_contact_cartesian_compensation_enabled=True,
               unknown_approach_held_axes_feedforward_enabled=True)
    report = dict(execution_policy='held_axes_approach',
                  approach_convergence_policy='bounded_approach_max_6mm', position_tolerance_m=.006,
                  code='PREGRASP_MEASURED_CONVERGED', plan_id=plan.plan_id,
                  fixed_goal_pose_index=1, fixed_goal_pose=list(pose_values(plan.poses[1])),
                  epoch_ns=node._following_epoch_ns, sdk_stamp_ns=19_900_000_000,
                  accepted_stamp_ns=19_900_000_000,
                  model_sha256=hashlib.sha256(b'test robot description').hexdigest())
    def compensate(*args):
        node._last_fixed_endpoint_compensation_report = deepcopy(report)
        return True
    node._compensate_fixed_endpoint = mock.Mock(side_effect=compensate)
    node._contact_linear_service = mock.Mock(return_value=old_move)
    return node, plan, pose, cfg, old_move, report


def run(node, plan, pose, cfg, old_move):
    return node._plan_and_execute_pose(task.GraspStages.APPROACH_TARGET,
        'linear 6D approach', pose, old_move, 'approach',
        execution_plan=plan, gcfg=cfg, execute_pose=old_move)


def test_atomic_held_approach_spends_pose1_once_and_independently_checks_arrival():
    node, plan, pose, cfg, old_move, _ = case()
    assert run(node, plan, pose, cfg, old_move)
    node._compensate_fixed_endpoint.assert_called_once_with(plan, cfg, 1)
    node._contact_linear_service.assert_not_called()
    node._wait_for_motion_settle.assert_not_called()
    old_move.assert_not_called()
    node._set_contact_endpoint_precision.assert_not_called()
    node._record_and_validate_measured_endpoint.assert_called_once_with(
        pose, plan, cfg, 'bounded 6D approach', required=True)
    assert node._wait_for_measured_endpoint_contract.call_args.args[1:3] == (.006, 5.)
    assert node.set_state.call_args_list == [
        mock.call(task.GraspStages.APPROACH_TARGET, 'planning bounded 6D approach'),
        mock.call(task.GraspStages.APPROACH_TARGET, 'moving bounded 6D approach')]


@pytest.mark.parametrize('fault', ['gateway_failure', 'wrong_policy',
    'wrong_plan', 'wrong_index', 'wrong_goal', 'planned_only', 'unstable', 'record_failed'])
def test_failed_held_approach_never_falls_back_or_starts_second_episode(fault):
    node, plan, pose, cfg, old_move, report = case()
    if fault == 'gateway_failure':
        node._compensate_fixed_endpoint.side_effect = None
        node._compensate_fixed_endpoint.return_value = False
    if fault == 'wrong_policy': report['execution_policy'] = 'cartesian'
    if fault == 'wrong_plan': report['plan_id'] = 'different'
    if fault == 'wrong_index': report['fixed_goal_pose_index'] = 2
    if fault == 'wrong_goal': report['fixed_goal_pose'][2] += .001
    if fault == 'planned_only': report['code'] = 'PREGRASP_PLANNED_ONLY'
    if fault == 'unstable': node._wait_for_measured_endpoint_contract.return_value = (False, .02, 2.)
    if fault == 'record_failed': node._record_and_validate_measured_endpoint.return_value = False
    assert not run(node, plan, pose, cfg, old_move)
    assert node._compensate_fixed_endpoint.call_count == 1
    old_move.assert_not_called()
    node._contact_linear_service.assert_not_called()
    node._set_contact_endpoint_precision.assert_not_called()


@pytest.mark.parametrize('value', ['true', 1, None, {}, []])
def test_invalid_held_approach_configuration_cannot_send_motion(value):
    node, plan, pose, cfg, old_move, _ = case()
    cfg['unknown_approach_held_axes_feedforward_enabled'] = value
    assert not run(node, plan, pose, cfg, old_move)
    node._compensate_fixed_endpoint.assert_not_called()
    old_move.assert_not_called()


def test_disabled_policy_retains_cartesian_approach_and_original_endpoint_compensation():
    node, plan, pose, cfg, old_move, _ = case()
    cfg['unknown_approach_held_axes_feedforward_enabled'] = False
    assert run(node, plan, pose, cfg, old_move)
    assert [call.args[1] for call in old_move.call_args_list] == [False, True]
    node._compensate_fixed_endpoint.assert_called_once_with(plan, cfg, 1)


def test_final_contact_descent_cannot_use_approach_policy():
    node, plan, pose, cfg, old_move, _ = case()
    pose.pose = deepcopy(plan.poses[2])
    assert node._plan_and_execute_pose(task.GraspStages.APPROACH_TARGET,
        'linear 6D grasp pose', pose, old_move, 'grasp',
        execution_plan=plan, gcfg=cfg, execute_pose=old_move)
    assert [call.args[1] for call in old_move.call_args_list] == [False, True]
    node._compensate_fixed_endpoint.assert_called_once_with(plan, cfg, 2)


@pytest.mark.parametrize('fault', ['epoch', 'model', 'stale_sdk', 'stale_encoder',
                                   'future', 'invalid_stamp', 'before_plan', 'epoch_during_settle'])
def test_report_or_final_context_change_cannot_authorize_next_stage(fault):
    node, plan, pose, cfg, old_move, report = case()
    if fault == 'epoch': report['epoch_ns'] += 1
    if fault == 'model': report['model_sha256'] = 'other'
    if fault == 'stale_sdk': report['sdk_stamp_ns'] = 19_000_000_000
    if fault == 'stale_encoder': report['accepted_stamp_ns'] = 19_000_000_000
    if fault == 'future': report['accepted_stamp_ns'] = 21_000_000_000
    if fault == 'invalid_stamp': report['sdk_stamp_ns'] = True
    if fault == 'before_plan': plan.header.stamp = task.rospy.Time.from_sec(19.95)
    if fault == 'epoch_during_settle':
        def change_epoch(*args):
            node._following_epoch_ns += 1
            return True, .004, 2.
        node._wait_for_measured_endpoint_contract.side_effect = change_epoch
    assert not run(node, plan, pose, cfg, old_move)
    assert node._compensate_fixed_endpoint.call_count == 1
    old_move.assert_not_called()


def test_unknown_10mm_general_policy_does_not_skip_precise_approach_gate():
    node, plan, pose, cfg, old_move, _ = case()
    cfg['measured_endpoint_position_tolerance_m'] = .010
    assert run(node, plan, pose, cfg, old_move)
    assert node._wait_for_measured_endpoint_contract.call_args.args[1:3] == (.006, 5.)


def test_final_record_uses_same_6mm_gate_without_changing_global_configuration():
    node, plan, pose, cfg, old_move, _ = case()
    cfg['measured_endpoint_position_tolerance_m'] = .010
    def final_sample(_pose, _plan, final_cfg, *_args, **_kw):
        return .008 <= final_cfg['measured_endpoint_position_tolerance_m']
    node._record_and_validate_measured_endpoint.side_effect = final_sample
    assert not run(node, plan, pose, cfg, old_move)
    assert cfg['measured_endpoint_position_tolerance_m'] == .010
    assert node._compensate_fixed_endpoint.call_count == 1


@pytest.mark.parametrize('field,value', [('position_tolerance_m', .010),
                                        ('approach_convergence_policy', 'legacy')])
def test_gateway_report_must_certify_same_precise_approach_policy(field, value):
    node, plan, pose, cfg, old_move, report = case()
    report[field] = value
    assert not run(node, plan, pose, cfg, old_move)
    node._wait_for_measured_endpoint_contract.assert_not_called()
