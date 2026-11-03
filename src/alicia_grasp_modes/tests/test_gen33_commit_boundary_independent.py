"""Independent current-source commit cancellation and original-limit checks."""
from copy import deepcopy
from types import SimpleNamespace

import pytest
from std_msgs.msg import Header

from test_initial_preflight_remote import initial
import test_gen33_initial_preflight_commit as owner


@pytest.mark.parametrize('event', ('false_object', 'manual_out_and_back',
    'control_epoch', 'sdk_out_and_back'))
def test_real_callbacks_cancel_an_admitted_lease_with_a_fresh_complete_join(initial, event):
    s = initial
    _, admission, row = owner.handoff(s)
    owner.deliver(s.n, row)
    owner.commit_time(s, admission)
    assert s.n._measurement_remote_active()
    assert s.n._ros_source_clock_ns() < row['object'].header.stamp.to_nsec() + admission['maximum_ns']
    if event == 'false_object':
        absent = deepcopy(row['object'])
        absent.header.stamp = owner.module.rospy.Time.from_sec(s.clock[0])
        absent.detected = False
        s.n.object_cb(absent)
    elif event == 'manual_out_and_back':
        s.n._initial_preflight_manual_cb(SimpleNamespace(data=True))
        s.n._initial_preflight_manual_cb(SimpleNamespace(data=False))
    elif event == 'control_epoch':
        ns = s.token['control_epoch_ns'] + 1
        s.n._measurement_remote_epoch_cb(Header(
            stamp=owner.module.rospy.Time(ns // 10**9, ns % 10**9),
            frame_id='sdk_reference_epoch'))
    else:
        owner.feedback(s, [.01, 0., 0., 0., 0., 0.])
        owner.feedback(s)
    reply = owner.request(s, 'commit')
    assert not reply['accepted']
    assert s.n._initial_preflight_remote['state'] != 'committed'
    assert s.token['token_id'] in s.n._initial_preflight_retired_tokens
    assert not s.n._measurement_remote_active()


@pytest.mark.parametrize('limit', ('confidence', 'jump', 'association'))
def test_current_source_uses_the_live_configured_quality_and_association_limits(initial, limit):
    s = initial
    lease, admission, row = owner.handoff(s)
    if limit == 'confidence':
        row['object'].confidence = .75
        s.params['/grasp']['min_object_confidence'] = .8
    else:
        row['object'].pose_base.pose.position.x += .01
        row['status']['position_base'][0] += .01
        if limit == 'jump':
            s.params['/grasp']['max_object_jump_m'] = .005
        else:
            s.n.target_instance_association_threshold_m = .005
    owner.deliver(s.n, row)
    owner.commit_time(s, admission)
    before = deepcopy(s.n.latest_object)
    reply = owner.request(s, 'commit')
    assert not reply['accepted']
    expected = 'confidence' if limit == 'confidence' else 'association'
    assert reply['reason'] == 'initial_preflight_current_object_' + expected
    assert s.n.latest_object == before
    assert s.n._measurement_remote_request == lease
    assert s.n._measurement_remote_reference == admission


def test_a_shorter_original_source_age_limit_is_not_expanded_to_point_eight(initial):
    s = initial
    assert owner.request(s)['accepted']
    # Finish a real in-flight producer frame before tightening the consumer
    # limit. Changing the producer's pinned policy mid-frame is a different
    # rejection; here admission freezes the real shorter limit before delivery.
    s.clock[0] += .2
    s.feed(target_depth=.35)
    row = owner.publication(s.p)
    owner.feedback(s)
    s.params.setdefault('/unknown_perception', {})['max_frame_age_sec'] = .6
    lease = owner.admit_measurement(s)
    admission = deepcopy(s.n._measurement_remote_reference)
    assert admission['maximum_ns'] == 600000000
    assert row['object'].detected and row['status']['state'] == 'ready'
    owner.deliver(s.n, row)
    owner.commit_time(s, admission, age=1.137253809)
    age = s.n._ros_source_clock_ns() - row['object'].header.stamp.to_nsec()
    assert admission['maximum_ns'] <= age < 800000000
    reply = owner.request(s, 'commit')
    assert not reply['accepted'] and reply['reason'] == 'TARGET_MEASUREMENT_PENDING'
    assert s.n._measurement_remote_request == lease
    assert s.n._measurement_remote_reference == admission
