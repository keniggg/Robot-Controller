"""Offline regression reproduction; no production edits, ROS master or motion.

Two synthetic RGB-D cases use the real station producer and typed retirement.
The recorded failure supplies only relative clock boundaries, not sensor data.
A passing test documents the current bug, not successful grasp acceptance.
"""
import json

import pytest

from test_station_compute_wait_producer import armed
from test_station_planning_handoff import cut_over
from test_locked_rgb_hold import custom_frame, latest_status, module


@pytest.mark.parametrize('after_reference_deadline', [False, True])
def test_typed_retirement_boundary_reproduces_current_failure(monkeypatch, after_reference_deadline):
    node, clock, joint, params, marker, previous_pair = armed(monkeypatch)
    reference = node._locked_rgb_hold_reference
    original_stamp = reference['source_stamp_ns']
    reference_sec = original_stamp * 1e-9
    original_last = node.tracker.last
    registration = node._station_compute_registration
    original_count = node.pubs['object'].publish.call_count
    retirement_seen = []
    original_cb = node._station_compute_state_cb

    def retire_while_reference_fresh(message):
        # Recorded task MOVE arrived roughly 793 ms after the old reference.
        clock[0] = reference_sec + .793
        joint(clock[0] - .001)
        original_cb(message)
        retirement = registration.get('retirement')
        assert retirement is not None, 'must exercise existing typed retirement'
        retirement_seen.append(retirement['stamp_ns'])

    monkeypatch.setattr(node, '_station_compute_state_cb', retire_while_reference_fresh)
    # custom_frame stamps images 100 ms before now; cut_over advances 200 ms.
    # Therefore this produces a source 400.645971 ms newer than the reference.
    clock[0] = reference_sec + .300645971

    def resume_after_transition():
        clock[0] = reference_sec + (.804669380 if after_reference_deadline else .795)
        joint(clock[0] - .0001)

    source = cut_over(monkeypatch, node, clock, joint, fault=resume_after_transition)
    now_ns = module.rospy.Time.now().to_nsec()
    source_ns = source[0].header.stamp.to_nsec()
    source_age = (now_ns - source_ns) * 1e-9
    reference_age = (now_ns - original_stamp) * 1e-9
    assert 0. < source_age < .8
    assert retirement_seen and retirement_seen[0] < original_stamp + 800_000_000
    assert node.last_good_source_stamp_ns == original_stamp
    assert node.tracker.last is original_last
    summary = dict(case='reference_expired' if after_reference_deadline else 'both_fresh',
        source_age_sec=source_age, reference_age_sec=reference_age,
        reference_ns=original_stamp, current_source_ns=source_ns,
        typed_retirement_ns=retirement_seen[0], validation_ns=now_ns,
        first_loss=node.first_loss, hardware_commands_issued=False)
    if after_reference_deadline:
        assert reference_age > .8
        assert node.first_loss is not None and node.tracker.lost
        status = latest_status(node)
        # cut_over pauses at post-segmentation, so the real arrival-reference
        # caller wraps the same lower-level freshness failure. GEN5 reached a
        # later tracking caller and exposed the lower-level text directly.
        assert status['frame_timing']['arrival_station_invalidated_reason'] == 'source_exceeds_existing_frame_limit'
        assert 'observation_arrival_station_invalidated_during_recovery' in status['detail']
        completion = status['frame_timing']['arrival_frame_completion']
        assert completion['qualified_entry_ns'] < completion['geometry_deadline_ns'] < now_ns
        assert now_ns < completion['completion_deadline_ns']
        summary['detail'] = status['detail']
        summary['underlying_arrival_station_invalidated_reason'] = status['frame_timing']['arrival_station_invalidated_reason']
        summary['injected_boundary'] = 'post_segment, before arrival reference revalidation'
        summary['completion'] = completion
        # A valid new image cannot undo the loss; merely discarding this error
        # would also leave the old reference stale before a new motion lease.
        clock[0] += .1
        custom_frame(node, clock, joint, target_depth=.35)
        assert node.first_loss is not None and node.tracker.lost
        assert latest_status(node)['detail'] == 'target_lost_requires_new_generation'
        summary['fresh_next_frame_detail'] = latest_status(node)['detail']
    else:
        assert reference_age < .8
        assert node.first_loss is None and not node.tracker.lost
        assert node.pubs['object'].publish.call_count == original_count
        assert node._locked_rgb_hold_reference is reference
    print('GEN5_RETIREMENT_REPRO ' + json.dumps(summary, sort_keys=True))
