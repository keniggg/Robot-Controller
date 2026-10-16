"""Independent source-bound completion after an actual feedback-gap quarantine.

GEN26 first-loss clock deltas run through production producer/consumer methods
with synthetic RGB-D. They do not claim replay of the historical image pixels.
"""
from copy import deepcopy
import itertools
import json
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest
from std_msgs.msg import Header, String

from test_initial_feedback_bracket_gap import scene as initial_scene, frame as gap_frame
from test_locked_rgb_hold import module
from test_unknown_pending_identity import publication, deliver


CAPTURE_DELTA = 654_324_770
START_DELTA = 663_007_736
FINISH_DELTA = 824_329_138


def recovery_case(monkeypatch, *, fault='', forward_admission=True):
    s = initial_scene.__wrapped__(monkeypatch)
    gap = gap_frame(s)
    deliver(s.node, gap)
    assert s.node._unknown_identity_pending is not None
    s.reference = s.p._locked_rgb_hold_reference
    old = s.reference['source_stamp_ns']
    source_ns = old+CAPTURE_DELTA
    s.source_ns = source_ns
    s.old_deadline = s.node._unknown_identity_pending['deadline_ns']
    s.intermediate, s.polls, s.refine_calls = [], [], []

    # Intermediate wire events reach the real consumer before computation
    # completes. The final measured triple is delivered separately in all
    # callback orders. No transport callback can manufacture geometry.
    def object_cb(message):
        if not message.detected:
            s.intermediate.append(('object', deepcopy(message)))
            if forward_admission:
                s.node.object_cb(deepcopy(message))
    def mask_cb(message):
        if not np.any(s.p.bridge.imgmsg_to_cv2(message, 'mono8')):
            s.intermediate.append(('mask', deepcopy(message)))
            if forward_admission:
                s.node.mask_cb(deepcopy(message))
    def status_cb(message):
        value = json.loads(message.data)
        if value.get('state') != 'ready':
            s.intermediate.append(('status', deepcopy(message)))
            if forward_admission:
                s.node._silhouette_source_status_cb(deepcopy(message))
    s.p.pubs['object'].publish.side_effect = object_cb
    s.p.pubs['object_mask'].publish.side_effect = mask_cb
    s.p.pubs['detector_status'].publish.side_effect = status_cb

    s.clock[0] = (old+(801_000_000 if fault == 'late_entry' else START_DELTA))/1e9
    s.joint((old+600_000_000)/1e9)
    original_lookup = s.p.buffer.lookup_transform
    waiting = []
    def lookup(*args, **kwargs):
        if args[0] == 'base_link' and not waiting:
            waiting.append(True)
            if fault != 'late_entry':
                s.clock[0] = (old+720_000_000)/1e9
            s.joint((old+700_000_000)/1e9)
        return original_lookup(*args, **kwargs)
    s.p.buffer.lookup_transform = lookup
    original_identity = module.metric_seeded_identity_evidence
    def finish(*args, **kwargs):
        result = original_identity(*args, **kwargs)
        s.refine_calls.append(True)
        finish_ns = source_ns+801_000_000 if fault == 'current_expired' else old+FINISH_DELTA
        s.clock[0] = finish_ns/1e9
        if fault == 'manual': s.params['/gui/joint_direct_mode'] = True
        elif fault == 'epoch':
            s.p._rgb_interest_epoch_cb(Header(stamp=module.rospy.Time.from_sec(s.clock[0]),
                                             frame_id='sdk_reference_epoch'))
        elif fault == 'out_and_back':
            s.joint(s.clock[0], [.006, 0., 0., 0., 0., 0.])
            s.joint(s.clock[0]+.001, [0.]*6)
        elif fault == 'camera': s.params['/camera']['fx'] += 1.
        elif fault == 'selection': s.params['/grasp_mode/selection']['generation'] += 1
        elif fault == 'config': s.p.config.support_threshold_m = .0019
        # Expiration is deliberately polled while the producer still owns
        # the in-flight source. A late result cannot resurrect an expired ID.
        s.node._unknown_identity_poll()
        s.polls.append(s.node._current_stream_target_identity())
        return result
    monkeypatch.setattr(module, 'metric_seeded_identity_evidence', finish)
    rgb = np.full((240, 320, 3), (60, 90, 190), np.uint8)
    rgb[106:134, 148:173] = 20
    depth = np.full((240, 320), .4, np.float32)
    depth[106:134, 148:173] = .35
    pair = (s.p.bridge.cv2_to_imgmsg(rgb, encoding='bgr8'),
            s.p.bridge.cv2_to_imgmsg(depth, encoding='32FC1'))
    for message in pair:
        message.header.stamp = module.rospy.Time(source_ns//10**9, source_ns%10**9)
        message.header.frame_id = s.p.camera_frame
    s.p.latest = pair
    s.p.process(None)
    s.recovery = publication(s.p)
    return s


@pytest.mark.parametrize('order', list(itertools.permutations(('object', 'mask', 'status'))))
def test_unique_timely_recovery_survives_original_deadline_and_all_final_callback_orders(monkeypatch, order):
    s = recovery_case(monkeypatch)
    assert s.refine_calls and s.polls == [s.identity]
    assert s.recovery['object'].detected, s.recovery['status']
    assert s.recovery['object'].header.stamp.to_nsec() == s.source_ns
    assert s.intermediate, 'completion must be established before the original pending deadline'
    assert not s.node.latest_object.detected and not s.node.geometry_pub.messages[-1].valid
    deliver(s.node, s.recovery, order)
    assert s.node._current_stream_target_identity() == s.identity
    assert s.node._unknown_identity_pending is None
    assert s.node.latest_object.detected
    assert s.node.latest_object.header.stamp.to_nsec() == s.source_ns
    assert not s.node.rich_plan_pub.messages[-1].valid


@pytest.mark.parametrize('fault', ['late_entry', 'current_expired', 'manual', 'epoch',
                                 'out_and_back', 'camera', 'selection', 'config'])
def test_completion_cannot_admit_late_or_changed_current_measurement(monkeypatch, fault):
    s = recovery_case(monkeypatch, fault=fault)
    assert not s.recovery['object'].detected
    assert not np.any(s.p.bridge.imgmsg_to_cv2(s.recovery['mask'], 'mono8'))
    assert s.p.last_good_source_stamp_ns == s.reference['source_stamp_ns']
    assert not s.node.geometry_pub.messages[-1].valid
    if fault == 'late_entry':
        assert not s.refine_calls


def test_missing_or_late_admission_never_resurrects_expired_consumer(monkeypatch):
    s = recovery_case(monkeypatch, forward_admission=False)
    assert s.refine_calls and s.polls[0] != s.identity
    retired = s.node._current_stream_target_identity()
    for kind, message in s.intermediate:
        if kind == 'status': s.node._silhouette_source_status_cb(deepcopy(message))
        else: getattr(s.node, kind+'_cb')(deepcopy(message))
    deliver(s.node, s.recovery)
    assert s.node._current_stream_target_identity() == retired
    assert not s.node.latest_object.detected and not s.node.geometry_pub.messages[-1].valid


def test_duplicate_processing_notices_cannot_extend_fixed_completion_or_revive_expiry(monkeypatch):
    s = recovery_case(monkeypatch)
    assert s.refine_calls and s.polls == [s.identity]
    assert s.recovery['object'].detected
    def deadlines(value, prefix=''):
        found = {}
        if isinstance(value, dict):
            for key, child in value.items():
                path = prefix+'/'+str(key)
                if 'deadline' in str(key) and isinstance(child, (int, float)):
                    found[path] = child
                found.update(deadlines(child, path))
        return found
    original = deadlines(s.node._unknown_identity_pending)
    assert original
    s.clock[0] += .01
    for kind, message in s.intermediate:
        if kind == 'status': s.node._silhouette_source_status_cb(deepcopy(message))
        else: getattr(s.node, kind+'_cb')(deepcopy(message))
    assert s.node._current_stream_target_identity() == s.identity
    assert deadlines(s.node._unknown_identity_pending) == original
    # Regardless of any stricter fixed cap, the existing two-source limit
    # cannot outlive source+.8 or old-reference+1.6 seconds.
    s.clock[0] = min(s.source_ns+800_000_000,
                     s.reference['source_stamp_ns']+1_600_000_000)/1e9 + .001
    s.node._unknown_identity_poll()
    retired = s.node._current_stream_target_identity()
    assert retired != s.identity
    for kind, message in s.intermediate:
        if kind == 'status': s.node._silhouette_source_status_cb(deepcopy(message))
    deliver(s.node, s.recovery)
    assert s.node._current_stream_target_identity() == retired
    assert not s.node.latest_object.detected


def test_processing_admission_cannot_authorize_another_measured_source(monkeypatch):
    s = recovery_case(monkeypatch)
    assert s.recovery['object'].detected
    forged = deepcopy(s.recovery)
    ns = s.source_ns+1
    for kind in ('object', 'mask'):
        forged[kind].header.stamp = module.rospy.Time(ns//10**9, ns%10**9)
    for key in ('source_stamp_ns', 'color_stamp_ns', 'depth_stamp_ns', 'tf_lookup_stamp_ns'):
        if key in forged['status']: forged['status'][key] = ns
    forged['status']['source_qualification']['source_stamp_ns'] = ns
    deliver(s.node, forged)
    assert not s.node.latest_object.detected
    assert not s.node.geometry_pub.messages[-1].valid


@pytest.mark.parametrize('owner', ('remote', 'collector', 'task'))
@pytest.mark.parametrize('clock_fault', ('frozen_ros', 'rollback_ros'))
def test_completion_uses_fixed_monotonic_budget_and_rejects_ros_rollback(monkeypatch, owner, clock_fault):
    from test_gen27_feedback_completion_consumers import task_case
    from alicia_grasp_modes import unknown_pending_identity as remote_module
    from alicia_grasp_modes import initial_preflight_recovery as collector_module
    from alicia_grasp_modes import initial_preflight_task as task_module

    s, c = task_case(monkeypatch)
    assert s.recovery['object'].detected
    collector = c.node._initial_preflight_collector
    notices = [message for kind, message in s.intermediate
               if kind == 'status' and json.loads(message.data).get('source_qualification', {}).get(
                   'state') == 'feedback_recovery_processing']
    assert notices
    clock = [time.monotonic()]
    for target in (remote_module, collector_module, task_module):
        monkeypatch.setattr(target, 'time', NS(monotonic=lambda: clock[0]), raising=False)

    def repeat_notice():
        for message in notices:
            if owner == 'remote':
                s.node._silhouette_source_status_cb(deepcopy(message))
            elif owner == 'collector':
                collector.status(json.loads(message.data), round(s.clock[0]*1e9))
            else:
                c.node._unknown_support_status_cb(deepcopy(message))

    # Observe the completion at the final ROS time before testing a rollback.
    # Repeating at a later monotonic time must preserve the first fixed budget.
    repeat_notice()
    if clock_fault == 'frozen_ros':
        clock[0] += .5
        repeat_notice()
        clock[0] += .5
    else:
        s.clock[0] -= .03

    if owner == 'remote':
        deliver(s.node, s.recovery)
        assert s.node._current_stream_target_identity() != s.identity
        assert not s.node.latest_object.detected
    elif owner == 'collector':
        now = round(s.clock[0]*1e9)
        collector.object(deepcopy(s.recovery['object']), now)
        collector.mask(deepcopy(s.recovery['mask']), now)
        collector.status(deepcopy(s.recovery['status']), now)
        assert s.source_ns not in collector.recoveries
        assert collector.fatal
    else:
        from test_initial_preflight_task import feed
        feed(c, s.recovery)
        assert c.node._execution_authority_revoked
        assert not c.node._initial_preflight_attempt['physical_action_submitted']


def test_delayed_duplicate_processing_notice_does_not_reopen_completed_recovery(monkeypatch):
    from test_gen27_feedback_completion_consumers import task_case
    from test_initial_preflight_task import feed
    from alicia_grasp_modes import unknown_pending_identity as remote_module
    from alicia_grasp_modes import initial_preflight_recovery as collector_module
    from alicia_grasp_modes import initial_preflight_task as task_module

    s, c = task_case(monkeypatch)
    deliver(s.node, s.recovery)
    feed(c, s.recovery)
    assert s.node._unknown_identity_pending is None
    assert c.node._initial_preflight_pending is None
    assert not c.node._execution_authority_revoked
    clock = time.monotonic()+2.
    for target in (remote_module, collector_module, task_module):
        monkeypatch.setattr(target, 'time', NS(monotonic=lambda: clock), raising=False)
    for kind, message in s.intermediate:
        if kind == 'status':
            # task_case also forwards this actual wire callback to the task.
            s.node._silhouette_source_status_cb(deepcopy(message))
    assert s.node._current_stream_target_identity() == s.identity
    assert s.node._unknown_identity_pending is None
    assert c.node._initial_preflight_pending is None
    assert not c.node._initial_preflight_collector.fatal
    assert not c.node._execution_authority_revoked
    assert not c.node._initial_preflight_attempt['physical_action_submitted']


@pytest.mark.parametrize('owner', ('producer', 'remote', 'collector'))
def test_first_processing_notice_cannot_revive_wall_expired_reference_with_ros_frozen(monkeypatch, owner):
    from alicia_grasp_modes import unknown_pending_identity as remote_module
    from alicia_grasp_modes import initial_preflight_recovery as collector_module

    clock = [100.]
    proxy = NS(**{name: getattr(time, name) for name in dir(time) if not name.startswith('__')})
    proxy.monotonic = lambda: clock[0]
    targets = (module,) if owner == 'producer' else (remote_module, collector_module)
    for target in targets:
        monkeypatch.setattr(target, 'time', proxy, raising=False)
    original_scene = initial_scene.__wrapped__
    rejected_at_receipt = []
    def scene(m):
        s = original_scene(m)
        if owner == 'producer':
            # The existing scene fixture deliberately couples its monotonic
            # clock to ROS. Decouple only after acquisition, retaining that
            # clock's current domain and the reference's already pinned cap.
            clock[0] = module.time.monotonic()
            monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
            original_joint = s.joint
            def joint(stamp, *args):
                original_joint(stamp, *args)
                # The actual trailing bracket arrives while ROS still reports
                # a timely .72s entry, after the old reference's wall budget.
                if stamp > s.p.last_good_source_stamp_ns/1e9+.65:
                    clock[0] += 1.
            s.joint = joint
        else:
            original_status = s.node._silhouette_source_status_cb
            def status(message):
                is_notice = json.loads(message.data).get('source_qualification', {}).get(
                    'state') == 'feedback_recovery_processing'
                if is_notice:
                    clock[0] += 1.
                original_status(message)
                if is_notice:
                    rejected_at_receipt.append(s.node._current_stream_target_identity() != s.identity)
            s.node._silhouette_source_status_cb = status
        return s
    import test_gen27_feedback_gap_completion_independent as this_module
    monkeypatch.setattr(this_module, 'initial_scene', NS(__wrapped__=scene))
    if owner == 'collector':
        from test_gen27_feedback_completion_consumers import task_case
        s, c = task_case(monkeypatch)
        assert s.recovery['object'].detected
        assert c.node._initial_preflight_collector.fatal
        assert c.node._execution_authority_revoked
    else:
        s = recovery_case(monkeypatch)
        if owner == 'producer':
            assert not s.recovery['object'].detected
            assert not any(json.loads(msg.data).get('source_qualification', {}).get('state') ==
                           'feedback_recovery_processing' for kind, msg in s.intermediate if kind == 'status')
        else:
            assert s.recovery['object'].detected
            assert rejected_at_receipt == [True]
