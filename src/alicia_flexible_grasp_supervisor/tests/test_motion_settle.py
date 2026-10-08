"""Offline completion regression: only independent encoder windows can settle."""
from collections import deque
from copy import deepcopy
import threading

import pytest
from sensor_msgs.msg import JointState

from test_grasp_task_sequence import grasp_task_node as task
from alicia_flexible_grasp.robot.stationary_following import (
    ARM_NAMES, accepted_stationary_window,
)


def measured(stamp_ns, value=0.):
    msg = JointState()
    msg.header.stamp = task.rospy.Time(stamp_ns // 1_000_000_000,
                                      stamp_ns % 1_000_000_000)
    msg.header.frame_id = 'sdk_measured'
    msg.name = list(ARM_NAMES) + ['right_finger']
    msg.position = [value] + [0.] * 5 + [.05]
    return msg


def fixture(monkeypatch, mode='steady'):
    node = task.GraspTaskNode.__new__(task.GraspTaskNode)
    node.active = True
    node._following_evidence_lock = threading.RLock()
    node._following_epoch_ns = 9_000_000_000
    node._following_accepted_history = deque([measured(10_000_000_000)], maxlen=128)
    # The legacy heartbeat cache is deliberately irrelevant to all tests.
    node.latest_joint_state = measured(10_000_000_000)
    clock = {'ns': 10_000_000_000, 'polls': 0}
    params = {'/grasp/motion_settle_timeout_sec': 2.,
              '/grasp/motion_settle_min_sec': .8}
    monkeypatch.setattr(task.rospy, 'get_param', lambda key, default=None: params.get(key, default))
    monkeypatch.setattr(task.rospy.Time, 'now', staticmethod(
        lambda: measured(clock['ns']).header.stamp))
    monkeypatch.setattr(task.time, 'monotonic', lambda: (clock['ns']-10_000_000_000)*1e-9)
    monkeypatch.setattr(task.rospy, 'is_shutdown', lambda: False)
    monkeypatch.setattr(task.rospy, 'loginfo', lambda *a, **k: None)
    monkeypatch.setattr(task.rospy, 'logwarn', lambda *a, **k: None)

    def sleep(seconds):
        interval = round(seconds*1e9)
        clock['ns'] += interval
        clock['polls'] += 1
        value = 0.
        if mode == 'cached':
            return
        if mode == 'duplicate_source':
            node._following_accepted_history.append(measured(10_000_000_000))
            return
        if mode == 'slow_drift':
            value = clock['polls']*.001  # every adjacent delta passes old .0025 gate
        if mode == 'between_poll_oscillation':
            node._following_accepted_history.append(measured(clock['ns']-interval//2, .02))
        if mode == 'late_settling' and clock['ns'] < 10_750_000_000:
            value = .02 if clock['polls'] % 2 else 0.
        if mode == 'epoch_change':
            node._following_epoch_ns += 1
        node._following_accepted_history.append(measured(clock['ns'], value))
    monkeypatch.setattr(task.rospy, 'sleep', sleep)
    return node, clock, params


@pytest.mark.parametrize('mode', [
    'cached', 'duplicate_source', 'slow_drift', 'between_poll_oscillation',
])
def test_polled_values_cannot_masquerade_as_a_complete_stable_window(monkeypatch, mode):
    node, clock, _ = fixture(monkeypatch, mode)
    assert node._wait_for_motion_settle('after action') is False
    assert clock['ns'] == 12_000_000_000
    assert node._last_motion_settle_sample is None


def test_complete_independent_window_finishes_without_extra_consecutive_poll_delay(monkeypatch):
    node, clock, _ = fixture(monkeypatch)
    assert node._wait_for_motion_settle('after action') is True
    report = node._last_motion_settle_sample
    assert 10_800_000_000 <= clock['ns'] <= 10_850_000_000
    assert report['stationary_window_end_ns']-report['stationary_window_start_ns'] >= 800_000_000
    assert report['accepted_sample_count'] >= 3
    assert max(report['joint_peak_to_peak_rad']) == 0.


def test_minimum_time_is_a_stable_window_not_elapsed_time_since_action(monkeypatch):
    node, clock, _ = fixture(monkeypatch, 'late_settling')
    assert node._wait_for_motion_settle('after action') is True
    assert clock['ns'] >= 11_500_000_000


def test_epoch_change_cannot_finish_a_prior_motion(monkeypatch):
    node, clock, _ = fixture(monkeypatch, 'epoch_change')
    assert node._wait_for_motion_settle() is False
    assert clock['ns'] < 10_800_000_000


def test_disabled_timeout_does_not_certify_settling(monkeypatch):
    node, clock, params = fixture(monkeypatch)
    params['/grasp/motion_settle_timeout_sec'] = 0.
    assert node._wait_for_motion_settle() is False
    assert clock['polls'] == 0


def evaluate(messages, **kwargs):
    cfg = dict(now_sec=11., epoch_ns=9_000_000_000,
               after_ns=10_000_000_000, duration_sec=.8,
               position_epsilon_rad=.0025)
    cfg.update(kwargs)
    return accepted_stationary_window(messages, **cfg)


def test_missing_interval_cannot_be_filled_by_distant_identical_samples():
    rows = [measured(t) for t in (10_000_000_000, 10_200_000_000,
                                  10_600_000_000, 10_800_000_000)]
    with pytest.raises(ValueError, match='continuous accepted samples'):
        evaluate(rows)


def test_pre_action_window_is_not_reused_and_source_must_be_fresh():
    rows = [measured(10_000_000_000+i*100_000_000) for i in range(9)]
    assert evaluate(rows)['accepted_sample_count'] == 9
    with pytest.raises(ValueError, match='post-action'):
        evaluate(rows, after_ns=10_100_000_000)
    with pytest.raises(ValueError, match='stale'):
        evaluate(rows, now_sec=11.4)


def test_named_order_is_stable_but_nonfinite_arm_feedback_is_rejected():
    rows = [measured(10_000_000_000+i*100_000_000) for i in range(9)]
    baseline = evaluate(rows)
    for row in rows:
        row.name.reverse()
        row.position.reverse()
    assert evaluate(rows) == baseline
    bad = deepcopy(rows)
    bad[-1].position[-1] = float('nan')
    with pytest.raises(ValueError, match='invalid sdk_measured arm positions'):
        evaluate(bad)


@pytest.mark.parametrize('failed_stage', ['pregrasp', 'before approach', 'approach'])
def test_legacy_grasp_honors_settle_failure_before_following_motion_or_closure(monkeypatch, failed_stage):
    from unittest.mock import Mock
    from test_grasp_task_sequence import GraspTaskSequenceTest, FakeServiceResponse
    fixture = GraspTaskSequenceTest()
    node = task.GraspTaskNode.__new__(task.GraspTaskNode)
    node.active = True
    node.latest_obj = fixture._object()
    node._lookup_camera_pose_base = lambda: fixture._pose(0., 0., .2)
    node._current_tool_pose_base = lambda: fixture._pose(10., 0., .2)
    node._wait_for_motion_settle = lambda reason: reason != failed_stage
    node.set_state = Mock()
    node._command_gripper_position = Mock(return_value=True)
    node._close_gripper = Mock(return_value=(True, 'closed'))
    node._plan_and_execute_pose = Mock(return_value=True)
    pre = Mock(return_value=FakeServiceResponse())
    linear = Mock(return_value=FakeServiceResponse())
    services = {'/supervisor/move_to_pose': pre,
                '/supervisor/move_to_pose_linear': linear,
                '/supervisor/set_gripper': Mock(),
                '/supervisor/compliant_close': Mock()}
    monkeypatch.setattr(task.rospy, 'wait_for_service', lambda *a, **k: None)
    monkeypatch.setattr(task.rospy, 'ServiceProxy', lambda name, kind: services[name])
    monkeypatch.setattr(task.rospy, 'get_param', lambda name, default=None: default)
    monkeypatch.setattr(task.rospy.Time, 'now', staticmethod(lambda: task.rospy.Time(10)))
    monkeypatch.setattr(task.rospy, 'loginfo', lambda *a, **k: None)
    assert node.execute() is False
    node._close_gripper.assert_not_called()
    node._plan_and_execute_pose.assert_not_called()
    assert linear.call_count == (2 if failed_stage == 'approach' else 0)
    assert node._command_gripper_position.call_count == (0 if failed_stage == 'pregrasp' else 1)
    assert node.set_state.call_args.args[0] == task.GraspStages.FAILED
