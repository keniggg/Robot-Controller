"""Task wiring and XML-RPC evidence boundaries, with no real ROS master."""
from collections import deque
from copy import deepcopy
from types import SimpleNamespace
import threading
from unittest import mock
import xmlrpc.client

import pytest

from test_grasp_task_sequence import grasp_task_node as task
from test_stationary_following import fixture, XML


def node_and_plan():
    node = task.GraspTaskNode.__new__(task.GraspTaskNode)
    _, sdk, accepted = fixture()
    node._following_evidence_lock = threading.RLock()
    node._following_sdk_history = deque(sdk, maxlen=128)
    node._following_accepted_history = deque(accepted, maxlen=128)
    node._following_epoch_ns = 9_000_000_000
    node._following_fk_cache = None
    node._last_measured_endpoint_sample = {'old_endpoint': True}
    node.set_state = mock.Mock()
    plan = SimpleNamespace(plan_id='new-plan', header=SimpleNamespace(frame_id='base_link'),
                           diagnostic=task._FAR_FIELD_OBSERVATION_PLAN)
    return node, plan


def run(node, plan, monkeypatch, fail_publish=False):
    writes = []

    def publish(name, value):
        if fail_publish:
            raise RuntimeError('parameter server unavailable')
        # A nanosecond source must never overflow XML-RPC's 32-bit integers.
        xmlrpc.client.dumps((value,), allow_none=False)
        writes.append((name, deepcopy(value)))

    monkeypatch.setattr(task.rospy, 'set_param', publish)
    monkeypatch.setattr(task.rospy, 'get_param', lambda name, default=None: (
        XML if name == '/robot_description' else default))
    monkeypatch.setattr(task.rospy.Time, 'now', lambda: task.rospy.Time.from_sec(10.05))
    return node._record_reused_observation_following(plan), writes


def test_reuse_publishes_new_nonzero_following_evidence_without_fake_endpoint(monkeypatch):
    node, plan = node_and_plan()
    ok, writes = run(node, plan, monkeypatch)
    assert ok
    assert len(writes) == 1
    name, sample = writes[0]
    assert name == '/grasp_6d/runtime_execution_error'
    assert sample['plan_id'] == plan.plan_id
    assert sample['position_error_m'] > .002
    assert sample['epoch_ns'] == '9000000000'
    assert sample['sdk_stamp_ns'] == '10000000000'
    assert node._last_measured_endpoint_sample is None  # not retreat feedforward
    node.set_state.assert_not_called()


@pytest.mark.parametrize('mutation', [
    lambda n, p: n._following_sdk_history.clear(),
    lambda n, p: n._following_accepted_history.clear(),
    lambda n, p: setattr(n, '_following_epoch_ns', None),
    lambda n, p: setattr(p, 'diagnostic', task._CONTACT_EXECUTION_PLAN),
    lambda n, p: setattr(p.header, 'frame_id', 'camera_color_optical_frame'),
])
def test_missing_evidence_fails_without_overwriting_residual_with_zero(monkeypatch, mutation):
    node, plan = node_and_plan()
    mutation(node, plan)
    ok, writes = run(node, plan, monkeypatch)
    assert not ok
    assert writes == []
    assert node._last_measured_endpoint_sample is None
    assert 'REUSED_OBSERVATION_FOLLOWING_EVIDENCE_INVALID' in node.set_state.call_args[0][1]


def test_failed_publish_cannot_allow_near_field(monkeypatch):
    node, plan = node_and_plan()
    ok, writes = run(node, plan, monkeypatch, fail_publish=True)
    assert not ok and not writes


def test_epoch_callback_drops_pre_enable_samples_and_ignores_late_old_epoch():
    node, _ = node_and_plan()
    new_epoch = SimpleNamespace(frame_id='sdk_reference_epoch',
                                stamp=task.rospy.Time.from_sec(9.5))
    node.following_epoch_cb(new_epoch)
    assert node._following_epoch_ns == 9_500_000_000
    assert not node._following_sdk_history and not node._following_accepted_history
    _, sdk, accepted = fixture()
    node.following_sdk_cb(sdk[-1])
    node.accepted_joint_cb(accepted[-1])
    old_epoch = SimpleNamespace(frame_id='sdk_reference_epoch',
                                stamp=task.rospy.Time.from_sec(9.0))
    node.following_epoch_cb(old_epoch)
    assert node._following_epoch_ns == 9_500_000_000
    assert len(node._following_sdk_history) == 1
    assert len(node._following_accepted_history) == 1
    node.following_epoch_cb(SimpleNamespace(frame_id='bad', stamp=new_epoch.stamp))
    assert node._following_epoch_ns is None
    assert not node._following_sdk_history and not node._following_accepted_history


def test_epoch_change_during_calculation_rejects_publish(monkeypatch):
    node, plan = node_and_plan()
    original = task.stationary_following_error

    def compute(*args, **kwargs):
        sample = original(*args, **kwargs)
        node._following_epoch_ns += 1
        return sample

    monkeypatch.setattr(task, 'stationary_following_error', compute)
    ok, writes = run(node, plan, monkeypatch)
    assert not ok and not writes
