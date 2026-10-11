"""Real arrival callbacks may revoke a marker between processing boundaries."""
from copy import deepcopy

import pytest

import alicia_grasp_modes.station_compute_wait_producer as producer
from test_station_compute_wait_producer import armed
from test_observation_arrival_station import completed
from test_observation_measurement_unknown import send, ack
from test_locked_rgb_hold import custom_frame, latest_status, module


def revoke(n, clock, joint, params, change):
    if change == 'motion':
        # GEN8 actual two-Joint6-encoder-count departure, without a ROS node.
        joint(clock[0], [0., 0., 0., 0., 0., .006135923151543])
    elif change == 'manual':
        params['/gui/joint_direct_mode'] = True
    elif change == 'lease':
        request = deepcopy(n._observation_measurement_status['request'])
        for key in ('source_stamps_ns', 'settle'):
            request.pop(key, None)
        request.update(state='executing', lease_id='next_station', request_id='next_executing',
            stamp_ns=module.rospy.Time.now().to_nsec(), arrival_stamp_ns=0, capture_deadline_ns=0,
            view_source_stamp_ns=n.last_good_source_stamp_ns)
        send(n, request)
        assert ack(n)['accepted'], ack(n)
    else:
        raise AssertionError(change)


@pytest.mark.parametrize('change', ['motion', 'manual', 'lease'])
def test_callback_between_scope_guard_and_binding_is_named_station_rejection(monkeypatch, change):
    n, clock, joint, params, marker, _ = armed(monkeypatch)
    registration = n._station_compute_registration
    before = n.last_good_source_stamp_ns
    original = producer._candidate_fingerprint
    injected = []
    def interrupted(candidate):
        value = original(candidate)
        if not injected:
            injected.append(True)
            revoke(n, clock, joint, params, change)
        return value
    monkeypatch.setattr(producer, '_candidate_fingerprint', interrupted)
    with pytest.raises(ValueError, match='station_compute_wait_station_changed'):
        n._station_compute_scope(registration)
    assert injected and n.last_good_source_stamp_ns == before
    if change == 'motion':
        assert marker['invalidated_reason'] == 'movement_after_qualified_arrival'
        assert 'observation_arrival_retry' not in registration['reference']
    if change == 'lease':
        assert n._locked_rgb_hold_reference is None


@pytest.mark.parametrize('boundary', ['kwargs', 'fragment', 'completion'])
@pytest.mark.parametrize('change', ['motion', 'manual', 'lease'])
def test_real_process_callback_cannot_reuse_marker_or_emit_keyerror(monkeypatch, boundary, change):
    n, clock, _, joint, params, marker = completed(monkeypatch)
    before = n.last_good_source_stamp_ns
    old_last = n.tracker.last
    published_before = n.pubs['object'].publish.call_count
    injected = []
    def inject():
        if not injected:
            injected.append(True)
            revoke(n, clock, joint, params, change)
    if boundary == 'kwargs':
        original = n._arrival_rgb_spatial_kwargs
        def wrapped(reference, *args, **kwargs):
            if reference is not None:
                inject()
            return original(reference, *args, **kwargs)
        monkeypatch.setattr(n, '_arrival_rgb_spatial_kwargs', wrapped)
    elif boundary == 'completion':
        original = n._begin_station_frame_completion
        def wrapped(source, reference, kind):
            if kind == 'observation_arrival_retry':
                inject()
            return original(source, reference, kind)
        monkeypatch.setattr(n, '_begin_station_frame_completion', wrapped)
    else:
        original = module._matching_candidates
        def wrapped(*args, **kwargs):
            matches = original(*args, **kwargs)
            if n._processing_timing['stage'] == 'tracking':
                inject()
            return matches
        monkeypatch.setattr(module, '_matching_candidates', wrapped)
    clock[0] += .2
    custom_frame(n, clock, joint, target_depth=.35)
    assert injected
    assert n.last_good_source_stamp_ns == before
    assert n.tracker.last is old_last
    assert not any(call[0][0].detected for call in
                   n.pubs['object'].publish.call_args_list[published_before:])
    status = latest_status(n)
    assert "'observation_arrival_retry'" not in status['detail']
    if change != 'lease':
        assert 'observation_arrival_station_changed' in status['detail'], status
        assert status['target_lost'], status
    else:
        # A positively frozen new lease may consume this old in-flight frame.
        assert n._observation_measurement is not None
        assert n._locked_rgb_hold_reference is None


@pytest.mark.parametrize('change', ['motion', 'manual', 'lease'])
def test_revoke_after_completed_refine_never_publishes_old_station(monkeypatch, change):
    n, clock, joint, params, marker, _ = armed(monkeypatch)
    before = n.last_good_source_stamp_ns
    old_last = n.tracker.last
    published_before = n.pubs['object'].publish.call_count
    original = module.refine_candidate
    calls = []
    def interrupted(*args, **kwargs):
        result = original(*args, **kwargs)
        calls.append(True)
        revoke(n, clock, joint, params, change)
        return result
    monkeypatch.setattr(module, 'refine_candidate', interrupted)
    clock[0] += .2
    custom_frame(n, clock, joint, target_depth=.35)
    assert calls and n.last_good_source_stamp_ns == before and n.tracker.last is old_last
    assert not any(call[0][0].detected for call in
                   n.pubs['object'].publish.call_args_list[published_before:])
    assert "'observation_arrival_retry'" not in latest_status(n)['detail']


def test_normal_arrival_kwargs_scope_and_next_ready_keep_original_contract(monkeypatch):
    n, clock, joint, params, marker, _ = armed(monkeypatch)
    registration = n._station_compute_registration
    reference = n._locked_rgb_hold_reference
    source_ns = n.last_good_source_stamp_ns
    deadline = marker['session_deadline_ns']
    assert n._station_compute_scope(registration) is reference
    kwargs = n._arrival_rgb_spatial_kwargs(reference)
    assert kwargs['observation_arrival_retry'] is True
    assert kwargs['observation_spatial_reference'] is marker.get('spatial_reference')
    clock[0] += .2
    custom_frame(n, clock, joint, target_depth=.35)
    assert latest_status(n)['state'] == 'ready'
    assert n.last_good_source_stamp_ns > source_ns
    assert n._locked_rgb_hold_reference['observation_arrival_retry'] is marker
    assert marker['session_deadline_ns'] == deadline


@pytest.mark.parametrize('repeated', [False, True])
@pytest.mark.parametrize('change', ['motion', 'manual', 'lease'])
def test_ready_enrollment_cannot_restore_revoked_marker_or_repeat_contract(monkeypatch, repeated, change):
    n, clock, joint, params, marker, source = armed(monkeypatch)
    before = n.last_good_source_stamp_ns
    original_registration = n._station_compute_registration
    payload = latest_status(n)
    payload.pop('station_compute_wait', None)
    if not repeated:
        n._station_compute_registration = None
    original = producer.hashlib.sha256
    injected = []
    def interrupted(*args, **kwargs):
        result = original(*args, **kwargs)
        if not injected:
            injected.append(True)
            revoke(n, clock, joint, params, change)
        return result
    monkeypatch.setattr(producer.hashlib, 'sha256', interrupted)
    n._station_compute_status(payload, source)
    assert injected
    assert n._station_compute_registration is None
    assert 'station_compute_wait' not in payload
    assert n.last_good_source_stamp_ns == before
    if change == 'motion':
        assert 'observation_arrival_retry' not in original_registration['reference']


def test_replaced_marker_cannot_supply_kwargs_for_current_frame(monkeypatch):
    n, clock, _, joint, params, marker = completed(monkeypatch)
    reference = n._locked_rgb_hold_reference
    n._arrival_frame_completion = dict(reference=reference, marker=marker,
        marker_kind='observation_arrival_retry')
    reference['observation_arrival_retry'] = dict(marker, lease_id='replacement')
    with pytest.raises(ValueError, match='observation_arrival_station_changed'):
        n._arrival_rgb_spatial_kwargs(reference)
