"""Independent registered-station SDK ingress and fixed-deadline regression.

Fake transport/clocks; real qualification, scope, SDK callbacks and process.
The original failing two-case diagnostic is preserved on SanStand separately.
"""
from copy import deepcopy

import pytest
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from test_station_compute_wait_producer import armed
from test_locked_rgb_hold import custom_frame, latest_status, module


def _empty_registered_gap(monkeypatch):
    node, clock, joint, params, marker, source = armed(monkeypatch)
    registration = node._station_compute_registration
    contract = deepcopy(registration['contract'])
    clock[0] += 1.3
    assert module.rospy.Time.now().to_nsec() < contract['identity_deadline_ns']
    node.latest = None
    node.last_receive_wall = clock[0] - 1.3
    return node, clock, joint, params, marker, registration, contract


def _after_first_real_rejection(monkeypatch, node, inject):
    original = node._station_compute_idle_wait
    calls = []
    def interrupted_audit():
        result = original()
        calls.append(result)
        if len(calls) == 1:
            assert result is False  # The real scope, never a stubbed verdict.
            inject()
        return result
    monkeypatch.setattr(node, '_station_compute_idle_wait', interrupted_audit)
    return calls


def _assert_no_positive_object_since(node, index):
    assert all(call.args[0].detected is False
               for call in node.pubs['object'].publish.call_args_list[index:])


def test_current_sdk_and_empty_slot_use_only_original_fixed_identity_wait(monkeypatch):
    node, clock, joint, params, marker, registration, contract = _empty_registered_gap(monkeypatch)
    mono = registration['monotonic_deadline']
    source = node.last_good_source_stamp_ns
    count = node.pubs['object'].publish.call_count
    joint(clock[0] - .01)
    node.process(None)
    assert node.first_loss is None
    assert latest_status(node)['state'] == 'processing_deadline_pending'
    assert latest_status(node)['station_compute_wait'] == contract
    assert registration['monotonic_deadline'] == mono
    assert node.last_good_source_stamp_ns == source
    _assert_no_positive_object_since(node, count)


def test_sdk_callback_during_failed_audit_rechecks_then_requires_fresh_complete_measurement(monkeypatch):
    node, clock, joint, params, marker, registration, contract = _empty_registered_gap(monkeypatch)
    source = node.last_good_source_stamp_ns
    mono = registration['monotonic_deadline']
    count = node.pubs['object'].publish.call_count
    def fresh_sdk():
        joint(clock[0] - .01)
        assert node._station_compute_scope(registration) is registration['reference']
    calls = _after_first_real_rejection(monkeypatch, node, fresh_sdk)
    node.process(None)
    assert calls[0] is False
    assert node.first_loss is None
    assert node._station_compute_registration is registration
    assert registration['contract'] == contract
    assert registration['monotonic_deadline'] == mono
    assert node.last_good_source_stamp_ns == source
    _assert_no_positive_object_since(node, count)
    # A bounded next-tick audit is permitted; it is not a new measured source.
    node.process(None)
    assert node.first_loss is None
    assert latest_status(node)['state'] == 'processing_deadline_pending'
    assert latest_status(node)['station_compute_wait'] == contract
    assert node.last_good_source_stamp_ns == source
    _assert_no_positive_object_since(node, count)
    clock[0] += .2
    fresh = custom_frame(node, clock, joint, target_depth=.35)
    status = latest_status(node)
    assert status['state'] == 'ready', status
    assert status['station_compute_recovery'] == contract
    assert node.last_good_source_stamp_ns == fresh[0].header.stamp.to_nsec()
    assert status['station_compute_wait']['source_ns'] > source


@pytest.mark.parametrize('fault', ['motion', 'epoch', 'ros_expiry', 'monotonic_expiry',
                                   'wrong_frame', 'nan', 'missing_joint', 'future'])
def test_ingress_change_cannot_restore_station_after_real_fault(monkeypatch, fault):
    node, clock, joint, params, marker, registration, contract = _empty_registered_gap(monkeypatch)
    count = node.pubs['object'].publish.call_count
    source = node.last_good_source_stamp_ns
    def fault_ingress():
        if fault == 'motion':
            joint(clock[0] - .02, [.005, 0., 0., 0., 0., 0.])
            joint(clock[0] - .01)  # A return cannot erase measured excursion.
        elif fault == 'epoch':
            node._rgb_interest_epoch_cb(Header(stamp=module.rospy.Time(2),
                                              frame_id='sdk_reference_epoch'))
            joint(clock[0] - .01)
        elif fault == 'ros_expiry':
            clock[0] = contract['identity_deadline_ns'] * 1e-9 + .001
            joint(clock[0] - .01)
        elif fault == 'monotonic_expiry':
            joint(clock[0] - .01)
            expired = registration['monotonic_deadline'] + .001
            monkeypatch.setattr(module.time, 'monotonic', lambda: expired)
        else:
            message = JointState()
            message.header.frame_id = 'sdk_measured'
            message.header.stamp = module.rospy.Time.from_sec(clock[0] - .01)
            message.name = list(module.JOINT_NAMES)
            message.position = [0.] * 6
            if fault == 'wrong_frame': message.header.frame_id = 'sdk_commanded'
            elif fault == 'nan': message.position[0] = float('nan')
            elif fault == 'missing_joint':
                message.name.pop(); message.position.pop()
            elif fault == 'future':
                message.header.stamp = module.rospy.Time.from_sec(clock[0] + .01)
            node._rgb_interest_joint_cb(message)
        with pytest.raises(ValueError):
            node._station_compute_scope(registration)
    calls = _after_first_real_rejection(monkeypatch, node, fault_ingress)
    node.process(None)
    assert calls[0] is False
    node.process(None)
    _assert_no_positive_object_since(node, count)
    assert node.last_good_source_stamp_ns == source
    assert latest_status(node).get('state') != 'ready'
    if fault == 'motion':
        # Existing measured-motion policy may request three-frame station
        # requalification. It cannot retain the old qualified marker.
        assert marker.get('invalidated_reason') == 'movement_after_qualified_arrival'
        assert node._locked_rgb_hold_reference is None or (
            node._locked_rgb_hold_reference.get('observation_arrival_retry') is not marker)
        if node._station_compute_registration is registration:
            with pytest.raises(ValueError):
                node._station_compute_scope(registration)
    else:
        assert node.first_loss is not None
        assert node._station_compute_registration is None


@pytest.mark.parametrize('clock_kind', ['ros', 'monotonic'])
def test_sdk_refresh_cannot_defer_an_expired_fixed_support_identity_deadline(monkeypatch, clock_kind):
    from test_support_measurement_hold import support_gap_depth
    node, clock, joint, params, marker, source = armed(monkeypatch)
    clock[0] += .15
    custom_frame(node, clock, joint, depth_image=support_gap_depth())
    assert latest_status(node)['detail'] == 'support_measurement_pending_revalidation'
    registration = node._station_compute_registration
    contract = deepcopy(registration['contract'])
    fixed_deadline = registration['support_gap']['deadline_ns']
    assert fixed_deadline == contract['identity_deadline_ns']
    # Fault injection replaces only the SDK receipt cache; the support-gap
    # registration itself comes from real pixels and the original gates.
    # This negative test does not claim this cache ordering occurred in GEN21.
    node._rgb_interest_feedback.clear()
    joint(clock[0] - 1.)
    count = node.pubs['object'].publish.call_count
    def refreshed_but_expired():
        joint(clock[0] - .01)
        if clock_kind == 'ros':
            clock[0] = fixed_deadline * 1e-9 + .001
        else:
            deadline = registration['support_gap_monotonic'] + .001
            monkeypatch.setattr(module.time, 'monotonic', lambda: deadline)
    calls = _after_first_real_rejection(monkeypatch, node, refreshed_but_expired)
    node.latest = None
    node.last_receive_wall = clock[0] - 1.3
    node.process(None)
    assert calls[0] is False
    assert node.first_loss is not None
    assert node._station_compute_registration is None
    _assert_no_positive_object_since(node, count)
