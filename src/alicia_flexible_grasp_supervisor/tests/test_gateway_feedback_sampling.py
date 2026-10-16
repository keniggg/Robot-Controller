"""Gateway sampling retains owned source frames and reports actual buffer age."""
from collections import deque
import json
import threading

import pytest
from test_motion_gateway_controller_start import motion_gateway_node as gateway
from test_stationary_following import fixture, message


def make_node(monkeypatch, now=10.05):
    fk, sdk, accepted = fixture()
    node = gateway.MotionGateway.__new__(gateway.MotionGateway)
    node._endpoint_feedback_lock = threading.RLock()
    node._endpoint_feedback_history = {k: deque(maxlen=128) for k in ('sdk', 'accepted')}
    for kind, messages in [('sdk', sdk), ('accepted', accepted)]:
        for msg in messages:
            node._record_endpoint_feedback(kind, msg)
    clock = [now]
    monkeypatch.setattr(gateway.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(gateway.rospy, 'get_time', lambda: clock[0])
    monkeypatch.setattr(gateway.rospy, 'is_shutdown', lambda: False)
    monkeypatch.setattr(gateway.rospy, 'sleep', lambda seconds: clock.__setitem__(0, clock[0]+seconds))
    return node, fk, sdk, accepted, clock


def test_sampler_owns_callback_messages_and_keeps_one_window_during_new_callback(monkeypatch):
    node, fk, incoming_sdk, incoming_accepted, _ = make_node(monkeypatch)
    # Caller reuse of a delivered message must not change stored evidence.
    incoming_sdk[-1].position[0] = .9
    incoming_accepted[-1].position[1] = .9
    original = gateway.stationary_following_error
    def append_during_validation(model, sdk, accepted, **kwargs):
        newer = message(10.1, 'sdk_transmitted', [.1]*6)
        newer.header.stamp = gateway.rospy.Time(10, 100_000_000)
        node._record_endpoint_feedback('sdk', newer)
        return original(model, sdk, accepted, **kwargs)
    monkeypatch.setattr(gateway, 'stationary_following_error', append_during_validation)
    result = node._endpoint_following_snapshot(fk, 9_000_000_000, None,
                                               context_validator=lambda: None)
    assert result['sdk_stamp_ns'] == 10_000_000_000
    assert result['sdk_positions_rad'] == [0.]*6
    assert result['joint_error_sdk_counts'][1] == pytest.approx(-8)
    assert node._endpoint_feedback_history['sdk'][-1].header.stamp.to_nsec() == 10_100_000_000


@pytest.mark.parametrize('fault', ['stale', 'missing', 'future', 'old_epoch'])
def test_sampling_failure_keeps_freshness_gate_and_describes_last_buffer(monkeypatch, fault):
    node, fk, _, _, clock = make_node(monkeypatch, now=12. if fault=='stale' else 10.05)
    if fault == 'missing':node._endpoint_feedback_history['sdk'].clear()
    if fault == 'future':
        node._record_endpoint_feedback('sdk', message(20., 'sdk_transmitted', [0.]*6))
    epoch=10_020_000_000 if fault=='old_epoch' else 9_000_000_000
    started=clock[0]
    with pytest.raises(gateway.ObservationPathError) as error:
        node._endpoint_following_snapshot(fk, epoch, None, context_validator=lambda: None)
    assert 'FEEDBACK_UNAVAILABLE' in str(error.value)
    details=json.loads(str(error.value).split('; snapshot=',1)[1])
    assert details['epoch_ns'] == epoch
    assert details['sdk_count'] == len(node._endpoint_feedback_history['sdk'])
    assert details['sdk_latest_stamp_ns'] == (None if fault=='missing' else 20_000_000_000 if fault=='future' else 10_000_000_000)
    assert details['now_sec'] >= started
    assert clock[0]-started < 2.06


def test_evidence_expiring_during_fk_is_not_returned(monkeypatch):
    node, fk, _, _, clock = make_node(monkeypatch)
    original = gateway.stationary_following_error
    accepted_results = []

    def delayed_validation(*args, **kwargs):
        result = original(*args, **kwargs)
        accepted_results.append(result)
        # The measured sample is 0.14 s old at entry and 0.54 s old
        # when FK completes. The unchanged 0.5 s gate must still apply.
        clock[0] += .4
        return result

    monkeypatch.setattr(gateway, 'stationary_following_error', delayed_validation)
    with pytest.raises(gateway.ObservationPathError, match='FEEDBACK_UNAVAILABLE'):
        node._endpoint_following_snapshot(fk, 9_000_000_000, None,
                                         context_validator=lambda: None)
    assert len(accepted_results) == 1


@pytest.mark.parametrize('delay_stage', ['context', 'evidence'])
def test_original_sampling_deadline_rejects_late_success(monkeypatch, delay_stage):
    node, fk, _, _, clock = make_node(monkeypatch)
    # A stopped ROS clock must not extend the monotonic two-second budget.
    monkeypatch.setattr(gateway.rospy, 'get_time', lambda: 10.05)
    original = gateway.stationary_following_error
    validation_calls = []

    def validate_context():
        if delay_stage == 'context':
            clock[0] += 2.1

    def delayed_validation(*args, **kwargs):
        validation_calls.append(True)
        result = original(*args, **kwargs)
        if delay_stage == 'evidence':
            clock[0] += 2.1
        return result

    monkeypatch.setattr(gateway, 'stationary_following_error', delayed_validation)
    with pytest.raises(gateway.ObservationPathError, match='sampling deadline expired'):
        node._endpoint_following_snapshot(fk, 9_000_000_000, None,
                                         context_validator=validate_context)
    assert len(validation_calls) == (0 if delay_stage == 'context' else 1)


def test_context_revoked_during_validation_cannot_return_old_epoch_evidence(monkeypatch):
    node, fk, _, _, _ = make_node(monkeypatch)
    original = gateway.stationary_following_error
    revoked = [False]

    def validate_context():
        if revoked[0]:
            raise gateway.ObservationPathError('PREGRASP_AUTHORITY_CHANGED')

    def changed_context(*args, **kwargs):
        result = original(*args, **kwargs)
        revoked[0] = True
        return result

    monkeypatch.setattr(gateway, 'stationary_following_error', changed_context)
    with pytest.raises(gateway.ObservationPathError, match='PREGRASP_AUTHORITY_CHANGED'):
        node._endpoint_following_snapshot(fk, 9_000_000_000, None,
                                         context_validator=validate_context)
