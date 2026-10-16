"""Independent real callback ordering across the bootstrap movement lock."""
from copy import deepcopy
import threading

import pytest

from test_actuation_bootstrap_measurement_task import bootstrap
from test_task_modes import module as mode_task


def actual_mode_task(c):
    """Keep transport fakes; use the operational callback MRO and both locks."""
    n = c.node
    original_validation = n._validate_bound_plan_locked
    initialized = mode_task.ModeAwareGraspTaskNode.__new__(mode_task.ModeAwareGraspTaskNode)
    initialized._initialize_mode_boundary()
    for name, value in initialized.__dict__.items():
        n.__dict__.setdefault(name, value)
    n.__class__ = mode_task.ModeAwareGraspTaskNode
    # The original fixture's plan is complete for the base action validator;
    # mode/support policy is covered by its own suite, not fabricated here.
    n._validate_bound_plan_locked = original_validation
    n._mode_ready = True
    n._mode_selection = deepcopy(n._mode_bound_selection)
    c.params['/grasp'] = dict(c.gcfg, plan_validity_sec=120.)
    return n


@pytest.mark.parametrize('fault', [None, 'false', 'base_header', 'camera_header',
    'low_confidence', 'changed_center', 'new_source', 'already_stale_at_entry',
    'epoch', 'manual', 'selection', 'plan_digest', 'negative_ack', 'invalid_geometry',
    'ros_expired', 'mono_expired', 'revoked', 'source_validity_changed', 'plan_replaced'])
def test_positive_received_before_motion_but_delayed_by_mode_lock_does_not_revoke_bootstrap(bootstrap, fault):
    c = bootstrap
    n = actual_mode_task(c)
    message = deepcopy(n.latest_obj)
    message.header.stamp = message.pose_base.header.stamp = message.pose_camera.header.stamp = mode_task.rospy.Time(3, 900000000)
    if fault == 'false': message.detected = False
    elif fault == 'base_header': message.pose_base.header.stamp = mode_task.rospy.Time(3, 800000000)
    elif fault == 'camera_header': message.pose_camera.header.stamp = mode_task.rospy.Time(3, 800000000)
    elif fault == 'low_confidence': message.confidence = .1
    elif fault == 'changed_center': message.pose_base.pose.position.x += .5
    elif fault == 'new_source':
        message.header.stamp = message.pose_base.header.stamp = message.pose_camera.header.stamp = mode_task.rospy.Time(4)
    elif fault == 'already_stale_at_entry':
        message.header.stamp = message.pose_base.header.stamp = message.pose_camera.header.stamp = mode_task.rospy.Time(3)
    retired = []
    original_retirement = n._actuation_bootstrap_retired_object
    def retirement(msg, received_ns):
        result = original_retirement(msg, received_ns)
        retired.append(result)
        return result
    n._actuation_bootstrap_retired_object = retirement
    callback_entered = threading.Event()
    callback_completed = threading.Event()
    errors = []
    threads = []
    original_mode_lock = n._mode_task_lock
    main_thread = threading.get_ident()
    class CallbackEntryLock:
        def __enter__(self):
            # obj_cb samples received_ns before entering this exact boundary.
            if threading.get_ident() != main_thread:
                callback_entered.set()
            original_mode_lock.acquire()
            return self
        def __exit__(self, *args): original_mode_lock.release()
        def acquire(self): return original_mode_lock.acquire()
        def release(self): return original_mode_lock.release()
    n._mode_task_lock = CallbackEntryLock()
    n._mode_task_lock.acquire()
    lock_held = [True]
    def receive():
        try:
            n.obj_cb(message)
        except Exception as exc:
            errors.append(exc)
        finally:
            callback_completed.set()
    def before_ack(request):
        if request['state'] == 'executing':
            t = threading.Thread(target=receive)
            threads.append(t)
            t.start()
            assert callback_entered.wait(1.)
            assert not callback_completed.is_set()
    c.control['before_ack'] = before_ack
    def finish_prefix():
        # The source was timely at actual callback entry, before the prefix.
        # Another callback may hold the outer mode lock waiting for the plan
        # RLock. Release it only after the 3-second physical call has elapsed.
        if fault == 'epoch': n._following_epoch_ns += 1
        elif fault == 'manual': c.params['/gui/joint_direct_mode'] = True
        elif fault == 'selection': n._mode_selection = dict(n._mode_selection, generation=999)
        elif fault == 'plan_digest': n._bound_execution_plan_digest = 'b' * 64
        elif fault == 'negative_ack':
            req = n._actuation_bootstrap_context['request']
            n._observation_measurement_acks[(req['request_id'], 'remote')]['accepted'] = False
        elif fault == 'invalid_geometry':
            n.latest_target_geometry = deepcopy(c.plan.object_geometry)
            n.latest_target_geometry.valid = False
        elif fault == 'ros_expired': c.now[0] = 123_500_000_000
        elif fault == 'mono_expired': c.mono[0] = 200.
        elif fault == 'revoked': n._execution_authority_revoked = True
        elif fault == 'source_validity_changed': n._configured_plan_validity = lambda unused: 119.
        elif fault == 'plan_replaced': n._bound_execution_plan.plan_id = 'another-plan'
        lock_held[0] = False
        n._mode_task_lock.release()
    c.control['after_prefix'] = finish_prefix
    original_gate = n._automatic_actuation_gate
    def finish_delayed_callback(config):
        # The original action lock has now been released; schedule the waiting
        # callback before the subsequent handoff check, without fake evidence.
        assert callback_completed.wait(1.)
        assert not errors, errors
        return original_gate(config)
    n._automatic_actuation_gate = finish_delayed_callback
    try:
        result = c.run()
    finally:
        if lock_held[0]: n._mode_task_lock.release()
        for thread in threads: thread.join(timeout=1.)
    assert not errors, errors
    assert all(not thread.is_alive() for thread in threads)
    if fault is None:
        assert retired == [True]
        assert result, n._last_state_message
        assert not n._execution_authority_revoked
        assert [m['state'] for m in c.messages] == ['executing', 'requalifying', 'qualified']
    else:
        assert retired == [False]
        assert not result
        assert n._actuation_bootstrap_receipt is None


def test_queued_positive_after_bootstrap_completion_does_not_replace_new_geometry(bootstrap):
    c = bootstrap
    n = actual_mode_task(c)
    old = deepcopy(n.latest_obj)
    old.header.stamp = old.pose_base.header.stamp = old.pose_camera.header.stamp = mode_task.rospy.Time(3, 900000000)
    assert c.run(), n._last_state_message
    assert n._actuation_bootstrap_context is None
    assert n._actuation_bootstrap_retired_context is not None
    fresh, stamp = deepcopy(n.latest_obj), n.latest_obj_time
    assert n._actuation_bootstrap_retired_object(old, 4_000_000_000)
    assert n.latest_obj == fresh and n.latest_obj_time == stamp
    assert not n._execution_authority_revoked
    n._clear_observe_return_attempt()
    assert not n._actuation_bootstrap_retired_object(old, 4_000_000_000)
