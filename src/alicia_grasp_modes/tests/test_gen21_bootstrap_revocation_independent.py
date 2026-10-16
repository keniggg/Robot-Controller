"""Independent cross-layer bootstrap revocation, with no ROS or hardware I/O.

Real remote retirement publishes into the real gateway callback while the real
task callback waits on its action lock. The original prefix planner must reject
the retired context before its fake manipulator receives a trajectory.
"""
from copy import deepcopy
import json
import threading
from types import SimpleNamespace as NS

import pytest
from std_msgs.msg import Header, String

from alicia_flexible_grasp.robot.observation_path_guard import (
    CommittedObservationContexts, committed_plan_digest)
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
from alicia_grasp_modes import observe_return_measurement_remote as remote
from test_gen21_bootstrap_measurement_remote import bootstrap
from test_observation_measurement_remote import context, request, at
from test_observation_path_guard import rich_plan, target_for
from test_actuation_bootstrap import prefix_planner
from test_moveit_planner_pose_feedback import make_pose
from test_motion_gateway_controller_start import motion_gateway_node
from test_grasp_task_sequence import grasp_task_node


@pytest.mark.parametrize('fault', [
    'object_false', 'producer_lost', 'control_epoch', 'source_deadline',
    'monotonic_deadline', 'task_revoked',
])
def test_retirement_during_sync_reaches_gateway_before_blocked_task_callback(
        bootstrap, monkeypatch, fault):
    c = bootstrap
    n = c.node
    n.robot_execution_active = True

    # A complete committed geometry gives the real gateway an actual capture.
    plan = rich_plan()
    plan.header.stamp = at(c.request['view_source_stamp_ns'])
    plan.object_geometry.header = deepcopy(plan.header)
    plan.target_track_id = plan.object_geometry.target_track_id = c.identity.track_id
    plan.model_choice = 'unknown_tabletop'
    plan.plan_id = compute_plan_id(plan)
    for key in ('_observe_return_prefix', '_observe_return_origin_prefix', 'latest_rich_plan'):
        setattr(n, key, deepcopy(plan))
    n._promotion_controller().execution_plan_id = plan.plan_id
    c.request.update(origin_plan_id=plan.plan_id, view_plan_id=plan.plan_id,
                     plan_sha256=committed_plan_digest(plan))
    assert request(n, c.request)['accepted']

    gateway = motion_gateway_node.MotionGateway.__new__(motion_gateway_node.MotionGateway)
    gateway._observation_context_lock = threading.RLock()
    gateway._observation_contexts = CommittedObservationContexts()
    gateway._observation_contexts.set_active(True)
    gateway._observation_contexts.ingest(plan)
    capture = gateway._observation_contexts.capture(target_for(plan), c.clock[0], 120.)
    gateway._observation_task_sub = NS(get_num_connections=lambda: 1)
    gateway._gui_direct_mode = False
    monkeypatch.setattr(motion_gateway_node.rospy, 'get_time', lambda: c.clock[0])
    monkeypatch.setattr(motion_gateway_node.rospy, 'is_shutdown', lambda: False)
    # Explicit healthy control: this fixture must pass the exact gateway
    # liveness and frozen-capture checks before receiving a tombstone.
    gateway._require_live_observation_task()
    gateway._observation_contexts.validate_capture(*capture)
    assert not gateway._observation_contexts.blocked

    task = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
    task.active = True
    task.latest_grasp6d_plan = deepcopy(plan)
    task._bound_execution_plan = deepcopy(plan)
    task._execution_authority_revoked = False
    task._configured_plan_validity = lambda _config: 120.
    task._target_occlusion_allowed_locked = lambda: False
    action_lock = threading.RLock()
    callback_waiting = threading.Event()
    callback_finished = threading.Event()
    callback_threads, callback_errors, tombstones, events = [], [], [], []

    def guard():
        callback_waiting.set()
        return action_lock
    task._grasp6d_plan_guard = guard

    def task_transport(message):
        try:
            task.grasp6d_plan_cb(message)
        except BaseException as exc:
            callback_errors.append(exc)
        finally:
            callback_finished.set()

    original_publish = n.rich_plan_pub.publish
    def publish(message):
        original_publish(message)
        tombstones.append(deepcopy(message))
        gateway._committed_observation_plan_cb(deepcopy(message))
        events.append('gateway_received_tombstone')
        thread = threading.Thread(target=task_transport, args=(deepcopy(message),))
        callback_threads.append(thread)
        thread.start()
        assert callback_waiting.wait(1.)
        assert not callback_finished.is_set()
    n.rich_plan_pub.publish = publish

    def retire_during_sync():
        events.append('reference_sync')
        gateway._require_live_observation_task()
        gateway._observation_contexts.validate_capture(*capture)
        revocation_before = gateway._observation_contexts.revocation
        if fault == 'object_false':
            obj = deepcopy(c.ready['object']); obj.detected = False
            n.object_cb(obj)
        elif fault == 'producer_lost':
            status = deepcopy(c.ready['status']); status['target_lost'] = True
            n._silhouette_source_status_cb(String(data=json.dumps(status)))
        elif fault == 'control_epoch':
            n._measurement_remote_epoch_cb(Header(
                stamp=at(c.request['control_epoch_ns'] + 1), frame_id='sdk_reference_epoch'))
        elif fault == 'source_deadline':
            c.clock[0] = c.request['deadline_ns'] / 1e9 + .001
            n._observation_measurement_poll()
        elif fault == 'monotonic_deadline':
            cutoff = n._measurement_bootstrap_deadline_mono
            monkeypatch.setattr(remote.time, 'monotonic', lambda: cutoff)
            n._observation_measurement_poll()
        else:
            c.clock[0] += .001
            revoked = dict(c.request, state='revoked', request_id='task-cancel',
                stamp_ns=n._ros_source_clock_ns(), reason='independent cancel')
            request(n, revoked)
        assert tombstones and all(message.valid is False for message in tombstones)
        assert gateway._observation_contexts.blocked
        assert gateway._observation_contexts.revocation > revocation_before
        assert gateway._observation_contexts.active_scene is None
        assert not task._execution_authority_revoked
        return True, 'offline reference synchronization boundary'

    planner, manipulator = prefix_planner()
    planner.observation_path_guard_required = True
    def path_guard(path):
        events.append('prefix_path_guard')
        return gateway._validate_frozen_observation_path(capture, path, planner)
    planner.observation_path_validator = path_guard
    planner.move_to_pose = lambda *_args, **_kwargs: (True, 'offline cached path')
    gateway._ensure_planner = lambda: planner
    gateway._fresh_actuation_status = lambda: 'PENDING:POSITIVE_ENABLE_REQUESTED'
    gateway._ensure_trajectory_controllers_started = lambda: (True, 'offline controller')
    gateway._synchronize_trajectory_controller_to_feedback = retire_during_sync
    gateway._wait_for_stationary_arm_feedback = lambda: (True, 'offline stationary')

    try:
        with action_lock:
            outcome = gateway.handle_observation_actuation(NS(execute=True, target=make_pose()))
            assert not outcome.success
            assert 'active task publisher is unavailable' in outcome.message
            assert callback_waiting.is_set() and not callback_finished.is_set()
            assert not task._execution_authority_revoked
            assert not manipulator.executed_plans
            assert planner._last_pose_plan is None
            assert events.index('reference_sync') < events.index('gateway_received_tombstone')
            assert events.index('gateway_received_tombstone') < events.index('prefix_path_guard')
    finally:
        for thread in callback_threads:
            thread.join(2.)
    assert callback_threads and all(not thread.is_alive() for thread in callback_threads)
    assert not callback_errors
    assert callback_finished.is_set() and task._execution_authority_revoked
    assert not n._promotion_controller().has_execution
    assert n._unknown_identity_retired_selection == n._mode_selection
    assert not n._measurement_remote_active()
