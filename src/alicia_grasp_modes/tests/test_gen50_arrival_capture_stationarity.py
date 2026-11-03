"""Source-bound stopping through arrival capture; no robot or replayed pixels.

The original G49 fixture supplies actual integer source/feedback timestamps.
The producer tests use the existing synthetic RGB-D/identity fixture and real
measurement callbacks. Neither layer proves a historical grasp would succeed.
"""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from alicia_grasp_modes.observe_return_measurement import (
    CaptureStationarityPending, capture_stationary_window,
    extend_capture_stationarity, validate_capture_stationarity_policy)
from test_observation_measurement_unknown import frozen, next_frame, send, ack
from test_locked_rgb_hold import latest_status, module


POLICY = dict(version=1, duration_sec=.8, position_epsilon_rad=.0025, minimum_samples=3)
BASE = 10_000_000_000


def records(count=13, offset=0., start=BASE):
    return [(start+i*100_000_000, [offset, 0., 0., 0., 0., 0.]) for i in range(count)]


def window(rows, source=BASE+1_050_000_000, certificate=None):
    return capture_stationary_window(rows, policy=POLICY, source_ns=source,
        now_ns=max(ns for ns, _ in rows), after_ns=BASE, certificate=certificate)


@pytest.mark.parametrize('field,value', [('version', True), ('version', 2),
    ('duration_sec', 0), ('duration_sec', float('nan')),
    ('position_epsilon_rad', -.1), ('position_epsilon_rad', True),
    ('minimum_samples', 1), ('minimum_samples', 3.)])
def test_malformed_policy_cannot_silently_disable_capture(field, value):
    bad = dict(POLICY, **{field: value})
    with pytest.raises(ValueError):
        validate_capture_stationarity_policy(bad)


def test_complete_window_uses_unique_sources_and_owned_certificate():
    rows = records()
    expected = window(rows)
    actual = window(list(reversed(rows))+deepcopy(rows))
    assert actual == expected
    assert actual['settled_end_ns'] < BASE+1_050_000_000
    assert actual['checked_through_ns'] == rows[-1][0]
    rows[0][1][0] = 99.
    assert actual['minimum_rad'] == [0.]*6


@pytest.mark.parametrize('name,value', [('source_ns', float(BASE+1_050_000_000)),
    ('now_ns', float(BASE+1_200_000_000)), ('after_ns', True)])
def test_feedback_and_image_clocks_are_integer_nanoseconds(name, value):
    args = dict(policy=POLICY, source_ns=BASE+1_050_000_000,
        now_ns=BASE+1_200_000_000, after_ns=BASE)
    args[name] = value
    with pytest.raises(ValueError):
        capture_stationary_window(records(), **args)


def test_conflicting_duplicate_source_is_not_an_independent_sample():
    rows = records()
    rows.append((rows[-1][0], [.001, 0., 0., 0., 0., 0.]))
    with pytest.raises(ValueError, match='source_changed'):
        window(rows)


def test_out_and_back_inside_window_requires_a_new_complete_window():
    rows = records()
    rows[9] = (rows[9][0], [.003, 0., 0., 0., 0., 0.])
    with pytest.raises(CaptureStationarityPending):
        window(rows)
    later = rows+records(12, start=BASE+1_300_000_000)
    proof = window(later, source=BASE+2_350_000_000)
    assert proof['window_start_ns'] > rows[9][0]


def test_same_fixed_window_covers_all_three_frames_and_commit():
    first = window(records())
    second_rows = records(17)
    second_rows[-1] = (second_rows[-1][0], [.0015, 0., 0., 0., 0., 0.])
    second = window(second_rows, source=BASE+1_550_000_000, certificate=first)
    third_rows = deepcopy(second_rows)+records(5, offset=.003, start=BASE+1_700_000_000)
    with pytest.raises(CaptureStationarityPending):
        window(third_rows, source=BASE+2_050_000_000, certificate=second)


def test_published_certificate_distinguishes_new_motion_and_late_past_evidence():
    proof = window(records())
    proof['closed_ns'] = proof['checked_through_ns']
    assert extend_capture_stationarity(proof, proof['closed_ns']+100_000_000,
        [.003, 0., 0., 0., 0., 0.]) == proof
    late = extend_capture_stationarity(proof, proof['closed_ns']-50_000_000,
        [.003, 0., 0., 0., 0., 0.])
    assert late['unstable'] and proof['unstable'] is False
    with pytest.raises(ValueError, match='source_changed'):
        extend_capture_stationarity(proof, proof['closed_ns'],
            [.001, 0., 0., 0., 0., 0.])


def test_original_g49_feedback_defers_by_motion_time_without_selecting_support_results():
    original = json.loads((Path(__file__).parent/'fixtures/gen50_g49_arrival_inputs.json').read_text())
    request = next(x['message'] for x in original['original_requests']
                   if x['message']['state'] == 'requalifying')
    all_rows = [(x['source_stamp_ns'], x['position'][:6]) for x in original['feedback']]
    # Every source in the original status stream is evaluated, including bad
    # support fits between good frames. No RGB/depth/TF outcome chooses a frame.
    for status in original['status']:
        source = status['source_stamp_ns']
        if not request['arrival_stamp_ns'] < source < request['capture_deadline_ns']:
            continue
        after = next((ns for ns, _ in all_rows if ns >= source), None)
        if after is None:
            continue
        rows = [(ns, q) for ns, q in all_rows if ns <= after]
        try:
            proof = capture_stationary_window(rows, policy=POLICY, source_ns=source,
                now_ns=after, after_ns=request['settle']['stationary_window_start_ns'])
        except CaptureStationarityPending:
            proof = None
        if source <= 1792321796686522722:
            assert proof is None
        if proof is not None:
            assert proof['window_start_ns'] >= 1792321796114392082
    assert request['capture_deadline_ns'] == 1792321798873908758
    assert original['limits']  # The fixture explicitly preserves replay limits.


def arrive_with_policy(n, clock, joint, request):
    # The legacy RGB-D fixture writes n.latest directly, bypassing frame() and
    # its stream-receipt clock. Exercise the real ingress before processing
    # each synthetic pair; an idle poll after completed ACK must not invent
    # stream loss simply because the fixture skipped that callback.
    original_process = n.process
    def ingress_process(event):
        pair = n.latest
        if pair is not None:
            n.latest = None
            n.frame(*pair)
        return original_process(event)
    n.process = ingress_process
    clock[0] += 2.
    start = clock[0]-1.05
    for when in np.arange(start, clock[0]+.001, .05):
        joint(float(when))
    arrived = deepcopy(request)
    now = module.rospy.Time.now().to_nsec()
    arrived.update(state='requalifying', request_id='request2', stamp_ns=now,
        arrival_stamp_ns=now, capture_deadline_ns=now+3_000_000_000,
        settle=dict(epoch_ns=n._rgb_interest_epoch,
            stationary_window_start_ns=module.rospy.Time.from_sec(start).to_nsec(),
            stationary_window_end_ns=now, capture_stationarity_policy=deepcopy(POLICY)))
    send(n, arrived)
    assert ack(n)['accepted']
    return arrived


def test_sub_identity_motion_restarts_only_confirmation_and_can_requalify(monkeypatch):
    n, clock, _, joint, _, request = frozen(monkeypatch)
    request = arrive_with_policy(n, clock, joint, request)
    original = deepcopy(n._observation_measurement['request'])
    next_frame(n, clock, joint)
    assert len(n._observation_measurement['confirmation'].source_stamps_ns) == 1
    clock[0] += .07
    joint(clock[0], [.003, 0., 0., 0., 0., 0.])
    # The frame helper emits feedback at its previous clock value. Move past
    # the excursion so that it does not rewrite that original source stamp.
    clock[0] += .03
    next_frame(n, clock, joint)
    assert not n.tracker.lost and n.first_loss is None
    assert n._observation_measurement['confirmation'].source_stamps_ns == []
    assert n._observation_measurement['request'] == original
    assert latest_status(n)['observation_measurement']['reason'] == 'waiting_for_stationary_capture'
    for _ in range(6):
        clock[0] += .1
        joint(clock[0])
    stamps = []
    for _ in range(3):
        pair = next_frame(n, clock, joint)
        stamps.append(pair[0].header.stamp.to_nsec())
    assert ack(n)['qualified'], latest_status(n)
    assert ack(n)['source_stamps_ns'] == stamps
    assert n._observation_measurement['request'] == original


def test_original_identity_excursion_still_revokes_even_after_return(monkeypatch):
    n, clock, _, joint, _, request = frozen(monkeypatch)
    arrive_with_policy(n, clock, joint, request)
    next_frame(n, clock, joint)
    clock[0] += .05
    joint(clock[0], [.005, 0., 0., 0., 0., 0.])
    clock[0] += .05
    joint(clock[0])
    n.process(None)
    assert n.tracker.lost and n._observation_measurement is None
    assert 'movement_after_stationary_arrival' in ack(n)['reason']


@pytest.mark.parametrize('change', ['manual', 'epoch', 'camera', 'deadline'])
def test_bounded_capture_wait_never_hides_control_or_context_revocation(monkeypatch, change):
    n, clock, _, joint, params, request = frozen(monkeypatch)
    arrived = arrive_with_policy(n, clock, joint, request)
    next_frame(n, clock, joint)
    if change == 'manual':
        params['/gui/joint_direct_mode'] = True
    elif change == 'epoch':
        n._rgb_interest_epoch += 1
    elif change == 'camera':
        params['/camera']['fx'] += 1
    else:
        clock[0] += 3.1
    n.process(None)
    assert n.tracker.lost and n._observation_measurement is None
    assert not ack(n)['accepted']


@pytest.mark.parametrize('late_past', [False, True])
@pytest.mark.parametrize('completed', [False, True])
def test_qualified_ack_is_frozen_and_late_counterevidence_cannot_replace_sources(monkeypatch, late_past, completed):
    n, clock, _, joint, _, request = frozen(monkeypatch)
    request = arrive_with_policy(n, clock, joint, request)
    for _ in range(3):
        next_frame(n, clock, joint)
    closed = n._observation_measurement['capture_stationarity']['certificate']['closed_ns']
    if completed:
        clock[0] += .01
        done = dict(request, state='qualified', request_id='request3',
            stamp_ns=module.rospy.Time.now().to_nsec(), source_stamps_ns=ack(n)['source_stamps_ns'])
        send(n, done)
        assert n._observation_measurement is None
        assert n._locked_rgb_hold_reference['observation_arrival_retry'][
            'capture_stationarity_certificate']['closed_ns'] == closed
    receipt = deepcopy(ack(n))
    assert receipt['qualified']
    clock[0] += .05
    # Preserve integer nanoseconds, including the late historical proof edge;
    # do not round-trip a Unix source through float seconds.
    stamp_ns = closed if late_past else module.rospy.Time.now().to_nsec()
    n._rgb_interest_joint_cb(NS(header=NS(frame_id='sdk_measured',
        stamp=module.rospy.Time(stamp_ns//10**9, stamp_ns%10**9)),
        name=list(module.JOINT_NAMES), position=[.003, 0., 0., 0., 0., 0.]))
    n.process(None)
    if late_past:
        assert n.tracker.lost and n._observation_measurement is None
        if completed:
            assert ack(n) == receipt  # Completed ACK is never rewritten.
            assert 'observation_capture_published_feedback_changed' in latest_status(n)['detail']
        else:
            assert not ack(n)['accepted']
    else:
        assert not n.tracker.lost and ack(n) == receipt
        if not completed:
            assert n._observation_measurement['qualified_stamps'] == receipt['source_stamps_ns']


@pytest.mark.parametrize('excursion', [.003, .005])
def test_real_third_frame_commit_rechecks_feedback_after_expensive_preparation(monkeypatch, excursion):
    n, clock, _, joint, _, request = frozen(monkeypatch)
    request = arrive_with_policy(n, clock, joint, request)
    for _ in range(2):
        next_frame(n, clock, joint)
    count = n.pubs['object'].publish.call_count
    original = n._prepare_locked_rgb_hold_reference
    def prepare(*args, **kwargs):
        result = original(*args, **kwargs)
        clock[0] += .01
        joint(clock[0], [excursion, 0., 0., 0., 0., 0.])
        return result
    monkeypatch.setattr(n, '_prepare_locked_rgb_hold_reference', prepare)
    next_frame(n, clock, joint)
    assert not ack(n)['qualified']
    if excursion == .003:
        assert not n.tracker.lost
        assert n._observation_measurement['confirmation'].source_stamps_ns == []
        assert n._observation_measurement['request'] == request
        assert n.pubs['object'].publish.call_count == count
    else:
        assert n.tracker.lost and n._observation_measurement is None


def test_delayed_image_cannot_erase_or_extend_new_confirmation(monkeypatch):
    n, clock, _, joint, _, request = frozen(monkeypatch)
    arrive_with_policy(n, clock, joint, request)
    first = next_frame(n, clock, joint)[0].header.stamp.to_nsec()
    next_frame(n, clock, joint)
    pending = n._observation_measurement
    stamps = list(pending['confirmation'].source_stamps_ns)
    revision = pending['capture_stationarity']['revision']
    assert n._measurement_capture_stationary(pending, first) is None
    assert pending['confirmation'].source_stamps_ns == stamps
    assert pending['capture_stationarity']['revision'] == revision


def test_completed_marker_keeps_feedback_arriving_during_appearance_copy(monkeypatch):
    n, clock, _, joint, _, request = frozen(monkeypatch)
    request = arrive_with_policy(n, clock, joint, request)
    for _ in range(3):
        next_frame(n, clock, joint)
    closed = n._observation_measurement['capture_stationarity']['certificate']['closed_ns']
    original = module.FrozenObservationAppearance
    def feedback(stamp_ns, value):
        n._rgb_interest_joint_cb(NS(header=NS(frame_id='sdk_measured',
            stamp=module.rospy.Time(stamp_ns//10**9, stamp_ns%10**9)),
            name=list(module.JOINT_NAMES), position=[value, 0., 0., 0., 0., 0.]))
    def appearance(*args, **kwargs):
        result = original(*args, **kwargs)
        feedback(closed-25_000_000, -.001)
        return result
    monkeypatch.setattr(module, 'FrozenObservationAppearance', appearance)
    clock[0] += .01
    done = dict(request, state='qualified', request_id='request3',
        stamp_ns=module.rospy.Time.now().to_nsec(), source_stamps_ns=ack(n)['source_stamps_ns'])
    send(n, done)
    assert n._observation_measurement is None and not n.tracker.lost
    certificate = n._locked_rgb_hold_reference['observation_arrival_retry'][
        'capture_stationarity_certificate']
    assert certificate['minimum_rad'][0] == -.001 and certificate['closed_ns'] == closed
    receipt = deepcopy(ack(n))
    feedback(closed-10_000_000, .002)
    n.process(None)
    assert n.tracker.lost and ack(n) == receipt
