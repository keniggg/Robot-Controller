"""Independent station identity provenance and strict appearance boundaries.

Original producer callbacks, 3-frame confirmation and appearance gates execute.
Only existing offline transport/clock/geometry fixtures are used; no ROS master.
"""
from copy import deepcopy
import json
import threading

import numpy as np
import pytest

from test_observation_measurement_unknown import frozen, arrival, next_frame, send, ack
from test_observation_arrival_station import completed
from test_locked_rgb_hold import module, latest_status, locked
from alicia_grasp_modes.observation_appearance import FrozenObservationAppearance, match_observation_appearance


def before_completion(monkeypatch):
    n, clock, feed, joint, params, request = frozen(monkeypatch)
    premotion = n._observation_measurement['confirmation'].appearance_reference
    request = arrival(n, clock, joint, request)
    pairs = []
    for _ in range(3):
        pairs.append(next_frame(n, clock, joint))
        assert n._observation_measurement['confirmation'].appearance_reference is premotion
        assert 'observation_arrival_retry' not in (n._locked_rgb_hold_reference or {})
    stamps = ack(n)['source_stamps_ns']
    assert stamps == [p[0].header.stamp.to_nsec() for p in pairs]
    clock[0] += .01
    terminal = dict(request, state='qualified', request_id='independent-qualified',
        stamp_ns=module.rospy.Time.now().to_nsec(), source_stamps_ns=stamps)
    return n, clock, joint, params, terminal, pairs[-1], premotion


def test_actual_completion_freezes_exact_third_source_and_never_rolls(monkeypatch):
    n, clock, joint, params, terminal, pair, premotion = before_completion(monkeypatch)
    accepted = n._locked_rgb_hold_reference
    original_pixels = accepted['bgr'].copy()
    original_mask = accepted['mask'].copy()
    send(n, terminal)
    assert ack(n)['accepted'] and n._observation_measurement is None
    marker = accepted['observation_arrival_retry']; template = marker['station_appearance']
    assert template.source_ns == terminal['source_stamps_ns'][-1]
    assert template.source_ns > premotion.source_ns and template is not premotion
    assert np.array_equal(template.bgr, original_pixels)
    assert np.array_equal(template.mask > 0, original_mask > 0)
    assert marker['fragment_appearance_reference'] is premotion
    for _ in range(4):
        pair = next_frame(n, clock, joint)
        assert latest_status(n)['state'] == 'ready'
        current = n._locked_rgb_hold_reference
        assert current is not accepted and current['observation_arrival_retry'] is marker
        assert marker['station_appearance'] is template
        assert template.source_ns == terminal['source_stamps_ns'][-1]
        assert n.last_good_source_stamp_ns == pair[0].header.stamp.to_nsec()
    with pytest.raises(ValueError):
        accepted['bgr'][:] = 199
    accepted['bgr'] = np.full_like(accepted['bgr'], 199)
    accepted['mask'] = np.zeros_like(accepted['mask'])
    assert np.array_equal(template.bgr, original_pixels)
    assert np.array_equal(template.mask > 0, original_mask > 0)


@pytest.mark.parametrize('fault', ['source', 'manual', 'epoch', 'model', 'lease', 'expired'])
def test_invalid_qualified_transition_cannot_create_template(monkeypatch, fault):
    n, clock, joint, params, terminal, _, _ = before_completion(monkeypatch)
    if fault == 'source': terminal['source_stamps_ns'][-1] += 1
    elif fault == 'manual': params['/gui/joint_direct_mode'] = True
    elif fault == 'epoch': n._rgb_interest_epoch += 1
    elif fault == 'model': params['/robot_description'] = '<robot name="changed"/>'
    elif fault == 'lease': terminal['lease_id'] = 'another-lease'
    else: clock[0] += .8
    previous_ack_count = n._observation_measurement_ack_pub.publish.call_count
    send(n, terminal)
    # Malformed lease binding is rejected before a request exists to ACK; the
    # prior requalifying ACK must not be mistaken for a qualified response.
    if n._observation_measurement_ack_pub.publish.call_count != previous_ack_count:
        assert not ack(n)['accepted']
    assert 'station_appearance' not in ((n._locked_rgb_hold_reference or {}).get('observation_arrival_retry') or {})


def test_process_cannot_use_template_until_qualified_callback_completes(monkeypatch):
    n, clock, joint, params, terminal, _, _ = before_completion(monkeypatch)
    entered, started, finished = threading.Event(), threading.Event(), threading.Event()
    original_service = n._service_rgb_reference_request
    def first_process_step():
        entered.set()
        return original_service()
    monkeypatch.setattr(n, '_service_rgb_reference_request', first_process_step)
    errors, threads = [], []
    def worker():
        started.set()
        try: n.process(None)
        except Exception as error: errors.append(error)
        finally: finished.set()
    def publishing(message):
        payload = json.loads(message.data)
        if payload['state'] == 'qualified' and payload['accepted']:
            assert n._observation_measurement is not None
            thread = threading.Thread(target=worker); threads.append(thread); thread.start()
            assert started.wait(1.)
            assert not entered.wait(.02) and not finished.is_set()
    n._observation_measurement_ack_pub.publish.side_effect = publishing
    send(n, terminal)
    for thread in threads: thread.join(2.)
    assert len(threads) == 1 and finished.is_set() and not errors
    assert entered.is_set() and n._observation_measurement is None


def test_qualified_ack_transport_failure_retires_uncommitted_template(monkeypatch):
    n, clock, joint, params, terminal, _, _ = before_completion(monkeypatch)
    def publishing(message):
        payload = json.loads(message.data)
        if payload['state'] == 'qualified' and payload['accepted']:
            raise ValueError('independent_transport_failure')
    n._observation_measurement_ack_pub.publish.side_effect = publishing
    send(n, terminal)
    assert not ack(n)['accepted']
    assert n._observation_measurement is None and n.tracker.lost
    assert 'station_appearance' not in ((n._locked_rgb_hold_reference or {}).get('observation_arrival_retry') or {})


@pytest.mark.parametrize('fault', ['manual', 'epoch', 'model', 'source_expired', 'pose_return'])
def test_owner_and_source_are_rechecked_after_template_copy_before_ack(monkeypatch, fault):
    n, clock, joint, params, terminal, _, _ = before_completion(monkeypatch)
    original = module.FrozenObservationAppearance
    class InterruptedFreeze(original):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            if fault == 'manual': params['/gui/joint_direct_mode'] = True
            elif fault == 'epoch':
                from std_msgs.msg import Header
                n._rgb_interest_epoch_cb(Header(stamp=module.rospy.Time(2), frame_id='sdk_reference_epoch'))
            elif fault == 'model': params['/robot_description'] = '<robot name="changed-during-copy"/>'
            elif fault == 'source_expired': clock[0] += .8
            else:
                joint(clock[0], [.005, 0., 0., 0., 0., 0.])
                joint(clock[0]+.001, [0.]*6)
    monkeypatch.setattr(module, 'FrozenObservationAppearance', InterruptedFreeze)
    send(n, terminal)
    assert not ack(n)['accepted']
    assert 'station_appearance' not in ((n._locked_rgb_hold_reference or {}).get('observation_arrival_retry') or {})


@pytest.mark.parametrize('fault', ['manual', 'epoch', 'source_expired', 'hold_replaced', 'request_changed'])
def test_changes_while_waiting_for_real_input_commit_lock_reject_qualified_ack(monkeypatch, fault):
    n, clock, joint, params, terminal, _, _ = before_completion(monkeypatch)
    real = n.input_lock
    waiting, copied, resume = threading.Event(), threading.Event(), threading.Event()
    original = module.FrozenObservationAppearance
    class SignaledFreeze(original):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            copied.set()
            assert resume.wait(2.)
    monkeypatch.setattr(module, 'FrozenObservationAppearance', SignaledFreeze)
    main_ident = threading.get_ident()
    class WaitingLock:
        def __enter__(self):
            if threading.get_ident() != main_ident: waiting.set()
            real.acquire(); return self
        def __exit__(self, *args): real.release()
    n.input_lock = WaitingLock()
    errors = []
    def worker():
        try: send(n, terminal)
        except Exception as error: errors.append(error)
    thread = threading.Thread(target=worker); thread.start()
    held = False
    try:
        assert copied.wait(2.)
        real.acquire(); held = True
        waiting.clear(); resume.set()
        assert waiting.wait(2.)
        if fault == 'manual': params['/gui/joint_direct_mode'] = True
        elif fault == 'epoch':
            from std_msgs.msg import Header
            n._rgb_interest_epoch_cb(Header(stamp=module.rospy.Time(2), frame_id='sdk_reference_epoch'))
        elif fault == 'source_expired': clock[0] += .8
        elif fault == 'hold_replaced': n._locked_rgb_hold_reference = dict(n._locked_rgb_hold_reference)
        else: n._observation_measurement['request'] = dict(n._observation_measurement['request'], request_id='changed')
    finally:
        resume.set()
        if held: real.release()
        thread.join(3.)
    assert not thread.is_alive() and not errors
    assert not ack(n)['accepted']
    assert 'station_appearance' not in ((n._locked_rgb_hold_reference or {}).get('observation_arrival_retry') or {})


def test_ordinary_ready_has_no_qualified_station_template(monkeypatch):
    n, clock, feed, joint, params = locked(monkeypatch)
    clock[0] += .2; feed(target_depth=.35)
    assert latest_status(n)['state'] == 'ready'
    assert 'observation_arrival_retry' not in n._locked_rgb_hold_reference


def station_match(monkeypatch):
    n, clock, _, joint, _, marker = completed(monkeypatch)
    template = marker['station_appearance']
    stamp = template.source_ns + 1
    # Source-pair transport is inert here; the actual appearance implementation
    # and its strict template and whole-mask constraints execute.
    pair = [type('Message', (), {'header':type('Header', (), {'stamp':module.rospy.Time(stamp//10**9, stamp%10**9)})()})()]
    checks = []
    context = n._observation_marker_appearance_context(pair, marker, lambda:checks.append(True))
    return n, marker, template, context, checks


@pytest.mark.parametrize('fault', ['same_source', 'older_source', 'wrong_role', 'wrong_histogram', 'iou_disabled'])
def test_station_appearance_context_cannot_self_authorize_or_change_binding(monkeypatch, fault):
    n, marker, template, context, checks = station_match(monkeypatch)
    histogram = template.histogram
    if fault == 'same_source': context['source_ns'] = template.source_ns
    elif fault == 'older_source': context['source_ns'] = template.source_ns-1
    elif fault == 'wrong_role': context['reference_role'] = 'ordinary'
    elif fault == 'wrong_histogram': histogram = np.roll(histogram, 1, axis=0)
    else: context['same_station_mask_iou_minimum'] = .84
    with pytest.raises(ValueError, match='observation_station_appearance_context_invalid'):
        match_observation_appearance(template.bgr, template.mask, histogram, template.contour,
            rgb_valid=template.valid, appearance_context=context)


@pytest.mark.parametrize('fault', ['partial_outline', 'different_colour'])
def test_original_colour_and_full_outline_rejects_remain_strict(monkeypatch, fault):
    n, marker, template, context, checks = station_match(monkeypatch)
    rgb, mask = template.bgr.copy(), template.mask.copy()
    if fault == 'partial_outline':
        y, x = np.nonzero(mask)
        mask[y[:len(y)//3], x[:len(x)//3]] = 0
    else: rgb[mask > 0] = (230, 210, 30)
    report = match_observation_appearance(rgb, mask, template.histogram, template.contour,
        rgb_valid=template.valid, appearance_context=context)
    assert not report['appearance_match'] and not report['metric_geometry_authority']
    assert report['max_colour_distance'] == .4 and report['max_contour_distance'] == .4
    if fault == 'partial_outline':
        assert report['code'] == 'SAME_STATION_COMPLETE_OUTLINE_MISMATCH'
        assert report['same_station_mask_iou'] < .85


def test_template_from_newer_frame_cannot_replace_qualified_source(monkeypatch):
    n, clock, _, joint, _, marker = completed(monkeypatch)
    old = marker['station_appearance']
    next_frame(n, clock, joint)
    current = n._locked_rgb_hold_reference
    marker['station_appearance'] = FrozenObservationAppearance(current['bgr'], current['mask'],
        old.valid, current['source_stamp_ns'], current['histogram'], current['contour'])
    with pytest.raises(ValueError, match='observation_station_appearance_binding_changed'):
        n._observation_marker_appearance(marker)


@pytest.mark.parametrize('fault', ['lease', 'target', 'view', 'first_qualified_source'])
def test_old_template_cannot_be_attached_to_changed_station_provenance(monkeypatch, fault):
    n, clock, _, joint, _, marker = completed(monkeypatch)
    if fault == 'lease': marker['measurement_binding']['lease_id'] += '-changed'
    elif fault == 'target': marker['measurement_binding']['target_track_id'] += '-changed'
    elif fault == 'view': marker['measurement_binding']['view_index'] += 1
    else: marker['qualified_source_stamps_ns'][0] += 1
    with pytest.raises(ValueError, match='observation_station_appearance_binding_changed'):
        n._observation_marker_appearance(marker)


def test_new_observation_lease_requires_new_third_measurement_template(monkeypatch):
    n, clock, joint, params, terminal, _, _ = before_completion(monkeypatch)
    send(n, terminal)
    old = n._locked_rgb_hold_reference['observation_arrival_retry']['station_appearance']
    request = {k:v for k,v in terminal.items() if k not in ('source_stamps_ns','settle')}
    request.update(state='executing', lease_id='independent-lease2', request_id='independent-executing2',
        view_index=2, stamp_ns=module.rospy.Time.now().to_nsec(), arrival_stamp_ns=0, capture_deadline_ns=0,
        view_source_stamp_ns=n.last_good_source_stamp_ns)
    send(n, request)
    assert ack(n)['accepted'] and n._locked_rgb_hold_reference is None
    request = arrival(n, clock, joint, request)
    for _ in range(3): next_frame(n, clock, joint)
    stamps = ack(n)['source_stamps_ns']
    clock[0] += .01
    request.update(state='qualified', request_id='independent-qualified2',
        stamp_ns=module.rospy.Time.now().to_nsec(), source_stamps_ns=stamps)
    send(n, request)
    assert ack(n)['accepted']
    new = n._locked_rgb_hold_reference['observation_arrival_retry']['station_appearance']
    assert new is not old and new.source_ns == stamps[-1] and new.source_ns > old.source_ns
