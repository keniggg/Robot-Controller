"""The empty-slot watchdog commits loss against current ingress, atomically."""
import threading

import numpy as np
import pytest
from std_msgs.msg import Header

from test_locked_rgb_hold import locked, latest_status, module
from test_station_compute_wait_producer import armed


def pair_at(node, stamp):
    rgb = np.full((240, 320, 3), (60, 90, 190), np.uint8)
    rgb[106:134, 148:173] = 20
    depth = np.full((240, 320), .4, np.float32)
    depth[106:134, 148:173] = .35
    pair = (node.bridge.cv2_to_imgmsg(rgb, encoding='bgr8'),
            node.bridge.cv2_to_imgmsg(depth, encoding='32FC1'))
    for message in pair:
        message.header.stamp = module.rospy.Time.from_sec(stamp)
        message.header.frame_id = node.camera_frame
    return pair


def ingress_during_idle_check(monkeypatch, node, pair, before=None):
    """Real frame callback on another thread after process snapshots None."""
    enter, done = threading.Event(), threading.Event()
    errors = []
    original = node._station_compute_idle_wait

    def receive():
        try:
            assert enter.wait(2.)
            node.frame(*pair)
        except BaseException as exc:
            errors.append(exc)
        finally:
            done.set()

    def audit():
        if before is not None:
            before()
        enter.set()
        assert done.wait(2.), 'frame must remain independent of process lock'
        return original()

    thread = threading.Thread(target=receive)
    thread.start()
    try:
        with monkeypatch.context() as patch:
            patch.setattr(node, '_station_compute_idle_wait', audit)
            node.process(None)
    finally:
        enter.set()
        thread.join(2.)
    assert not thread.is_alive()
    assert not errors


def test_pair_arriving_after_empty_snapshot_is_processed_without_false_loss(monkeypatch):
    node, clock, _, joint, _ = locked(monkeypatch)
    node.last_receive_wall = clock[0] - 1.
    pair = pair_at(node, clock[0] - .1)
    joint(clock[0] - .15); joint(clock[0] - .05)
    reference, last = node._locked_rgb_hold_reference, node.tracker.last
    good_source = node.last_good_source_stamp_ns
    published = node.pubs['object'].publish.call_count

    ingress_during_idle_check(monkeypatch, node, pair)

    assert node.first_loss is None and not node.tracker.lost
    assert node.latest == pair
    assert node._locked_rgb_hold_reference is reference and node.tracker.last is last
    assert node.last_good_source_stamp_ns == good_source
    assert node.pubs['object'].publish.call_count == published
    assert node._latest_frame_timing['source_stamp_ns'] == pair[0].header.stamp.to_nsec()
    node.process(None)
    assert latest_status(node)['state'] == 'ready'
    assert node.pubs['object'].publish.call_args[0][0].detected
    assert node.last_good_source_stamp_ns == pair[0].header.stamp.to_nsec()


def test_late_pair_keeps_original_stamp_and_is_rejected_on_its_own_age(monkeypatch):
    node, clock, _, _, _ = locked(monkeypatch)
    old_good = node.last_good_source_stamp_ns
    clock[0] += 3.
    node.last_receive_wall = clock[0] - 3.
    pair = pair_at(node, clock[0] - 2.2)
    ingress_during_idle_check(monkeypatch, node, pair)
    assert node.first_loss is None
    assert node.latest == pair
    assert node._latest_frame_timing['ingress_source_age_sec'] == pytest.approx(2.2)

    node.process(None)
    assert node.tracker.lost
    assert node.first_loss['reason'] == 'perception_failure: source_frame_stale_or_future'
    assert node.first_loss['source_stamp_ns'] == pair[0].header.stamp.to_nsec()
    assert node.last_good_source_stamp_ns == old_good
    assert not node.pubs['object'].publish.call_args[0][0].detected


def test_registered_station_with_stale_feedback_does_not_misclassify_new_ingress(monkeypatch):
    node, clock, joint, _, _, _ = armed(monkeypatch)
    registration = node._station_compute_registration
    source = node.last_good_source_stamp_ns
    clock[0] += 1.3  # Real scope check rejects the old SDK feedback.
    node.last_receive_wall = clock[0] - 1.3
    pair = pair_at(node, clock[0] - .1)
    ingress_during_idle_check(monkeypatch, node, pair)
    assert node.first_loss is None and not node.tracker.lost
    assert node._station_compute_registration is registration
    assert node.last_good_source_stamp_ns == source
    assert node.latest == pair
    # New actual feedback plus a complete fresh RGB-D measurement is still
    # required; the empty-slot check itself grants neither.
    joint(clock[0] - .15); joint(clock[0] - .05)
    node.process(None)
    assert latest_status(node)['state'] == 'ready'
    assert node.last_good_source_stamp_ns == pair[0].header.stamp.to_nsec()


def test_recorded_late_pair_then_sdk_refresh_preserves_pending_and_fresh_recovery(monkeypatch):
    node, clock, joint, _, _, _ = armed(monkeypatch)
    registration = node._station_compute_registration
    contract = dict(registration['contract'])
    mono = registration['monotonic_deadline']
    old_source = node.last_good_source_stamp_ns
    reference = node._locked_rgb_hold_reference
    # GEN21: last READY was ~3.8s old; an exact pair arrived ~2.2s
    # after its own source, shortly before the source-less loss decision.
    # The original five-second identity window still has time remaining.
    clock[0] += 3.7
    node.last_receive_wall = clock[0] - 3.7
    pair = pair_at(node, clock[0] - 2.2)
    ingress_during_idle_check(monkeypatch, node, pair)
    assert node.first_loss is None
    assert node._station_compute_registration is registration
    assert node.last_good_source_stamp_ns == old_source

    joint(clock[0] - .05, [0.] * 6)
    node.process(None)
    status = latest_status(node)
    assert status['state'] == 'processing_deadline_pending', status
    assert status['source_stamp_ns'] == pair[0].header.stamp.to_nsec()
    assert status['station_compute_wait'] == contract
    assert node._station_compute_registration is registration
    assert registration['monotonic_deadline'] == mono
    assert node._locked_rgb_hold_reference is reference
    assert node.last_good_source_stamp_ns == old_source
    assert node.first_loss is None and not node.tracker.lost
    assert not node.pubs['object'].publish.call_args[0][0].detected

    clock[0] += .2
    fresh = pair_at(node, clock[0] - .1)
    joint(clock[0] - .15, [0.] * 6); joint(clock[0] - .05, [0.] * 6)
    node.frame(*fresh)
    node.process(None)
    status = latest_status(node)
    assert status['state'] == 'ready', status
    assert status['station_compute_recovery'] == contract
    assert node.last_good_source_stamp_ns == fresh[0].header.stamp.to_nsec()
    assert node.first_loss is None and not node.tracker.lost
    assert node.pubs['object'].publish.call_args[0][0].detected


@pytest.mark.parametrize('change', ['epoch', 'motion', 'manual'])
def test_new_ingress_does_not_requalify_a_revoked_station(monkeypatch, change):
    node, clock, joint, params, _, _ = armed(monkeypatch)
    node.last_receive_wall = clock[0] - 1.
    clock[0] += .2
    pair = pair_at(node, clock[0] - .1)
    joint(clock[0] - .15); joint(clock[0] - .05)
    good_source = node.last_good_source_stamp_ns
    published = node.pubs['object'].publish.call_count

    def revoke():
        if change == 'epoch':
            node._rgb_interest_epoch_cb(Header(stamp=module.rospy.Time(2), frame_id='sdk_reference_epoch'))
        elif change == 'motion':
            joint(clock[0], [.005, 0., 0., 0., 0., 0.])
        else:
            params['/gui/joint_direct_mode'] = True

    ingress_during_idle_check(monkeypatch, node, pair, before=revoke)
    # Arrival is not measurement or a new station lease. The next real process
    # must still run all original scope checks before publishing geometry.
    node.process(None)
    if change == 'motion':
        # Existing measured-motion recovery may retain identity, never the
        # revoked station or its geometry. It still needs its own new lease
        # and three-frame qualification; arrival alone cannot complete it.
        assert node._station_requalification_hold is not None
        assert node._observation_measurement is None
        assert (node._locked_rgb_hold_reference or {}).get('observation_arrival_retry') is None
        assert latest_status(node)['state'] == 'pending'
        assert node.pubs['object'].publish.call_count == published
    else:
        assert node.first_loss is not None and node.tracker.lost
        assert not node.pubs['object'].publish.call_args[0][0].detected
    assert node.last_good_source_stamp_ns == good_source


@pytest.mark.parametrize('receive_age,expected_loss', [(1., True), (.2, False)])
def test_no_new_pair_preserves_original_stream_age_gate(monkeypatch, receive_age, expected_loss):
    node, clock, _, _, _ = locked(monkeypatch)
    node.last_receive_wall = clock[0] - receive_age
    node.process(None)
    assert (node.first_loss is not None) is expected_loss
    assert node.tracker.lost is expected_loss
    if expected_loss:
        assert node.first_loss['reason'] == 'exact_rgbd_stream_stale'


@pytest.mark.parametrize('invalid', ['mismatched', 'consumed', 'before_selection'])
def test_rejected_pair_cannot_reset_the_empty_stream_watchdog(monkeypatch, invalid):
    node, clock, _, _, _ = locked(monkeypatch)
    node.last_receive_wall = clock[0] - 1.
    pair = pair_at(node, clock[0] - .1)
    if invalid == 'mismatched':
        pair[1].header.stamp = module.rospy.Time.from_sec(clock[0] - .2)
    else:
        stamp = node.last_consumed_ns if invalid == 'consumed' else node.selection_ns
        for message in pair:
            message.header.stamp = module.rospy.Time(stamp // 10**9, stamp % 10**9)
    ingress_during_idle_check(monkeypatch, node, pair)
    assert node.latest is None
    assert node.last_receive_wall == clock[0] - 1.
    assert node.first_loss['reason'] == 'exact_rgbd_stream_stale'


def test_terminal_no_input_decision_holds_same_lock_as_frame_callback(monkeypatch):
    node, clock, _, _, _ = locked(monkeypatch)
    node.last_receive_wall = clock[0] - 1.
    original = node.unavailable
    held = []

    def unavailable(*args, **kwargs):
        held.append(node.input_lock._is_owned())
        return original(*args, **kwargs)

    monkeypatch.setattr(node, 'unavailable', unavailable)
    node.process(None)
    assert held == [True]
    assert node.first_loss['reason'] == 'exact_rgbd_stream_stale'


@pytest.mark.parametrize('registered', [False, True])
def test_stream_loss_preserves_scope_diagnostic_without_changing_deadlines(monkeypatch, registered):
    if registered:
        node, clock, _, _, _, _ = armed(monkeypatch)
        registration = node._station_compute_registration
        contract = dict(registration['contract'])
        mono = registration['monotonic_deadline']
    else:
        node, clock, _, _, _ = locked(monkeypatch)
        contract, mono = {}, None
    clock[0] += 1.3
    node.last_receive_wall = clock[0] - 1.3
    previous_feedback_ns = max(ns for ns, _ in node._rgb_interest_feedback)
    node.process(None)
    assert node.first_loss['reason'] == 'exact_rgbd_stream_stale'
    assert node.first_loss['source_stamp_ns'] is None
    audit = node.first_loss['candidate_evidence']['rgbd_idle_audit']
    assert audit['registration_present'] is registered
    assert audit['outcome'] == ('scope_rejected' if registered else 'no_registration')
    assert bool(audit['scope_failure_reason']) is registered
    assert audit['identity_deadline_ns'] == contract.get('identity_deadline_ns')
    assert audit['geometry_deadline_ns'] == contract.get('geometry_deadline_ns')
    assert audit['identity_deadline_monotonic_sec'] == mono
    assert audit['latest_feedback_stamp_ns'] == previous_feedback_ns
    assert audit['decision_ros_ns'] == module.rospy.Time.now().to_nsec()
    assert audit['decision_monotonic_sec'] == clock[0]
    assert audit['last_receive_monotonic_sec'] == clock[0] - 1.3
    assert audit['latest_pair_present'] is False and audit['diagnostic_only'] is True
    assert audit['maximum_frame_age_sec'] == .8
    assert node.tracker.lost and node._station_compute_registration is None
    assert latest_status(node)['first_loss'] == node.first_loss


@pytest.mark.parametrize('changed', [None, 'epoch', 'motion', 'identity_deadline'])
def test_sdk_refresh_after_failed_idle_audit_defers_only_the_intact_station(monkeypatch, changed):
    node, clock, joint, _, marker, _ = armed(monkeypatch)
    registration = node._station_compute_registration
    contract = dict(registration['contract'])
    mono = registration['monotonic_deadline']
    source = node.last_good_source_stamp_ns
    published = node.pubs['object'].publish.call_count
    clock[0] += 1.3
    node.last_receive_wall = clock[0] - 1.3
    original = node._station_compute_idle_wait

    def refreshed_after_audit():
        result = original()
        assert result is False
        audit = node._station_compute_idle_audit
        assert audit['scope_failure_reason'] == 'source_exceeds_existing_frame_limit'
        assert audit['failed_feedback_stamp_ns'] < module.rospy.Time.now().to_nsec() - 800_000_000
        joint(clock[0] - .01)
        if changed == 'epoch':
            node._rgb_interest_epoch += 1
        elif changed == 'motion':
            joint(clock[0], [.005, 0., 0., 0., 0., 0.])
        elif changed == 'identity_deadline':
            clock[0] = contract['identity_deadline_ns'] * 1e-9 + .001
            joint(clock[0] - .01)
        return result

    with monkeypatch.context() as patch:
        patch.setattr(node, '_station_compute_idle_wait', refreshed_after_audit)
        node.process(None)
    if changed is not None:
        assert node.first_loss is not None and node.tracker.lost
        assert not node.pubs['object'].publish.call_args[0][0].detected
        return
    assert node.first_loss is None and not node.tracker.lost
    assert node.pubs['object'].publish.call_count == published
    assert node._station_compute_registration is registration
    assert node.last_good_source_stamp_ns == source
    assert registration['contract'] == contract and registration['monotonic_deadline'] == mono
    assert node._locked_rgb_hold_reference['observation_arrival_retry'] is marker
    assert node._station_compute_idle_feedback_retry is None
    node.process(None)
    assert latest_status(node)['state'] == 'processing_deadline_pending'
    assert latest_status(node)['station_compute_wait'] == contract
    assert node.last_good_source_stamp_ns == source
    assert node.first_loss is None and not node.tracker.lost
