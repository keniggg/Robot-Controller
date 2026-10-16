"""Initial SDK synchronization crosses a measured identity handoff, offline."""
from copy import deepcopy
from unittest.mock import Mock

import numpy as np
import pytest

from test_locked_rgb_hold import locked, custom_frame, latest_status, module
from test_observation_measurement_unknown import send, ack, arrival, next_frame


# The first accepted feedback excursion in GEN20 attempt 1, before its goal.
SYNC_Q = [0., -.0015339807878856915, -.007669903939428208,
          .001533980787885636, 0., 0.]


def bootstrap_fixture(monkeypatch):
    n, clock, feed, joint, params = locked(monkeypatch, initial_station=True)
    params['/grasp'] = {'plan_validity_sec': 120.}
    n._observation_measurement_ack_pub = Mock()
    source = n.last_good_source_stamp_ns
    deadline = source + 120_000_000_000
    request = dict(version=1, state='executing', motion_kind='actuation_bootstrap',
        lease_id='bootstrap-lease', request_id='bootstrap-executing',
        session_id='bootstrap:test', selection=deepcopy(params['/grasp_mode/selection']),
        target_epoch=4, target_track_id='g1-t4', origin_plan_id='bound-plan',
        origin_snapshot_stamp_ns=source, view_plan_id='bound-plan',
        view_source_stamp_ns=source, view_index=1, plan_sha256='a' * 64,
        control_epoch_ns=n._rgb_interest_epoch, deadline_ns=deadline,
        stamp_ns=module.rospy.Time.now().to_nsec(),
        arrival_stamp_ns=0, capture_deadline_ns=0,
        actuation_bootstrap=dict(version=1, source_deadline_ns=deadline),
        metric_geometry_authority=False, motion_authorized=False)
    return n, clock, feed, joint, params, request


def begin(monkeypatch):
    values = bootstrap_fixture(monkeypatch)
    n, _, _, _, _, request = values
    send(n, request)
    assert ack(n)['accepted'], ack(n)
    return values


def moved_joint(joint):
    return lambda stamp, q=None: joint(stamp, SYNC_Q if q is None else q)


def test_unowned_sdk_sync_still_invalidates_original_station(monkeypatch):
    n, clock, _, joint, _, _ = bootstrap_fixture(monkeypatch)
    marker = n._locked_rgb_hold_reference['initial_station_current_support']
    joint(clock[0], SYNC_Q)
    assert marker['invalidated_reason'] == 'movement_after_initial_acquisition'
    clock[0] += .2
    custom_frame(n, clock, joint, target_depth=.35, q=SYNC_Q)
    assert n.tracker.lost and n.first_loss is not None
    assert not n.pubs['object'].publish.call_args[0][0].detected


def test_bootstrap_ack_retires_station_before_sync_without_publishing_moving_geometry(monkeypatch):
    n, clock, feed, joint, _, request = begin(monkeypatch)
    anchor, last, stamp = n.tracker.anchor, n.tracker.last, n.last_good_source_stamp_ns
    published = n.pubs['object'].publish.call_count
    assert n._locked_rgb_hold_reference is None
    assert n._arrival_frame_completion is None and n._station_compute_registration is None
    joint(clock[0], SYNC_Q)
    for _ in range(3):
        clock[0] += .2
        feed(q=SYNC_Q, target_depth=0.)
    assert n.pubs['object'].publish.call_count == published
    assert n.tracker.anchor is anchor and n.tracker.last is last
    assert n.last_good_source_stamp_ns == stamp
    assert not n.tracker.lost and n.first_loss is None
    assert latest_status(n)['state'] == 'pending'
    assert n._observation_measurement['request'] == request


@pytest.mark.parametrize('stage', ['segment', 'refine', 'new_hold'])
def test_inflight_premotion_frame_is_retired_after_explicit_handoff(monkeypatch, stage):
    n, clock, feed, joint, _, request = bootstrap_fixture(monkeypatch)
    stamp, last = n.last_good_source_stamp_ns, n.tracker.last
    published = n.pubs['object'].publish.call_count
    owner, name = ((module, 'segment_with_support_fallback') if stage == 'segment' else
                   (module, 'refine_candidate') if stage == 'refine' else
                   (n, '_prepare_locked_rgb_hold_reference'))
    original = getattr(owner, name)
    done = []
    def handoff(*args, **kwargs):
        result = original(*args, **kwargs)
        if not done:
            done.append(True)
            send(n, request)
            assert ack(n)['accepted'], ack(n)
            joint(clock[0], SYNC_Q)
        return result
    monkeypatch.setattr(owner, name, handoff)
    feed(target_depth=.35)
    assert done == [True]
    assert n.pubs['object'].publish.call_count == published
    assert n.last_good_source_stamp_ns == stamp and n.tracker.last is last
    assert n._locked_rgb_hold_reference is None
    assert n._observation_measurement is not None
    assert not n.tracker.lost and n.first_loss is None


def test_new_station_needs_three_real_frames_and_accepts_the_next_ordinary_view(monkeypatch):
    n, clock, _, joint, params, request = begin(monkeypatch)
    anchor, plane = n.tracker.anchor, n.tracker.plane_base.copy()
    published = n.pubs['object'].publish.call_count
    measured = moved_joint(joint)
    request = arrival(n, clock, measured, request)
    stamps = []
    for index in range(3):
        pair = next_frame(n, clock, measured)
        stamps.append(pair[0].header.stamp.to_nsec())
        assert n.pubs['object'].publish.call_count == published + (index == 2)
    assert ack(n)['qualified'] and ack(n)['source_stamps_ns'] == stamps
    clock[0] += .01
    send(n, dict(request, state='qualified', request_id='bootstrap-qualified',
        stamp_ns=module.rospy.Time.now().to_nsec(), source_stamps_ns=stamps))
    assert ack(n)['accepted'] and ack(n)['qualified']
    assert n._observation_measurement is None
    marker = n._locked_rgb_hold_reference['observation_arrival_retry']
    np.testing.assert_array_equal(marker['arrival_q'], SYNC_Q)
    assert 'initial_station_current_support' not in n._locked_rgb_hold_reference
    assert n.tracker.anchor is anchor and not n.tracker.lost
    np.testing.assert_array_equal(n.tracker.plane_base, plane)
    # The next ordinary tracked frame uses the new station and does not rearm
    # the original pose. No synthetic epoch/view promotion is manufactured.
    clock[0] += .2
    custom_frame(n, clock, joint, target_depth=.35, q=SYNC_Q)
    assert latest_status(n)['state'] == 'ready', latest_status(n)
    assert n._locked_rgb_hold_reference['observation_arrival_retry'] is marker
    ordinary = dict(request, state='executing', motion_kind='observation',
        lease_id='view-lease', request_id='view-executing', session_id='real-session',
        stamp_ns=module.rospy.Time.now().to_nsec(), deadline_ns=int((clock[0] + 60.) * 1e9),
        view_source_stamp_ns=n.last_good_source_stamp_ns,
        arrival_stamp_ns=0, capture_deadline_ns=0)
    ordinary.pop('actuation_bootstrap')
    ordinary.pop('settle')
    send(n, ordinary)
    assert ack(n)['accepted'], ack(n)
    assert n._observation_measurement['request']['target_epoch'] == 4
    assert n._observation_measurement['request']['target_track_id'] == 'g1-t4'
    assert n._observation_measurement['request']['view_index'] == 1
    assert n._locked_rgb_hold_reference is None


@pytest.mark.parametrize('change', ['missing_station', 'station_invalidated', 'moved',
                                  'deadline_extension', 'validity_change'])
def test_bootstrap_cannot_admit_unqualified_initial_station_or_extended_source(monkeypatch, change):
    n, clock, _, joint, params, request = bootstrap_fixture(monkeypatch)
    marker = n._locked_rgb_hold_reference['initial_station_current_support']
    if change == 'missing_station':
        n._locked_rgb_hold_reference.pop('initial_station_current_support')
    elif change == 'station_invalidated': marker['invalidated_reason'] = 'movement_after_initial_acquisition'
    elif change == 'moved': joint(clock[0], SYNC_Q)
    elif change == 'deadline_extension':
        request['deadline_ns'] += 1
        request['actuation_bootstrap']['source_deadline_ns'] += 1
    else: params['/grasp']['plan_validity_sec'] = 60.
    send(n, request)
    assert not ack(n)['accepted'], ack(n)
    assert getattr(n, '_observation_measurement', None) is None
    assert 'bootstrap-lease' not in getattr(n, '_observation_measurement_seen', ())


@pytest.mark.parametrize('change', ['manual', 'epoch', 'selection', 'prior_loss',
                                  'validity_change', 'source_expired', 'monotonic_expired'])
def test_bootstrap_custody_never_overrides_revocation_or_original_deadline(monkeypatch, change):
    n, clock, _, _, params, request = begin(monkeypatch)
    if change == 'manual': params['/gui/joint_direct_mode'] = True
    elif change == 'epoch': n._rgb_interest_epoch += 1
    elif change == 'selection': params['/grasp_mode/selection']['generation'] += 1
    elif change == 'prior_loss': n.tracker.lost = True
    elif change == 'validity_change': params['/grasp']['plan_validity_sec'] = 119.
    elif change == 'source_expired':
        deadline = request['deadline_ns']
        expired = module.rospy.Time(deadline // 10**9, deadline % 10**9)
        monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda: expired))
    else:
        fixed = module.rospy.Time.now()
        monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda: fixed))
        clock[0] = n._observation_measurement['deadline_monotonic']
    n.process(None)
    assert n._observation_measurement is None and n.tracker.lost
    assert not ack(n)['accepted']


def test_post_arrival_excursion_still_revokes_bootstrap_requalification(monkeypatch):
    n, clock, _, joint, _, request = begin(monkeypatch)
    measured = moved_joint(joint)
    arrival(n, clock, measured, request)
    next_frame(n, clock, measured)
    changed = list(SYNC_Q); changed[0] += .005
    joint(clock[0] + .01, changed)
    joint(clock[0] + .02, SYNC_Q)
    n.process(None)
    assert n.tracker.lost and n._observation_measurement is None
    assert 'movement_after_stationary_arrival' in ack(n)['reason']


def test_wrong_target_at_new_station_cannot_qualify_bootstrap(monkeypatch):
    n, clock, _, joint, _, request = begin(monkeypatch)
    measured = moved_joint(joint)
    arrival(n, clock, measured, request)
    published, last_good = n.pubs['object'].publish.call_count, n.last_good_source_stamp_ns
    for _ in range(4): next_frame(n, clock, measured, shift=80)
    assert not ack(n)['qualified']
    assert n.pubs['object'].publish.call_count == published
    assert n.last_good_source_stamp_ns == last_good
    assert not n._observation_measurement.get('qualified_stamps')
