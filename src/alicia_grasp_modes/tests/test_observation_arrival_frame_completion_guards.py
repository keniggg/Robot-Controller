"""Independent bounds for one normal arrival frame's computation lifetime."""
from copy import deepcopy

import numpy as np
import pytest

from test_observation_arrival_station import completed
from test_locked_rgb_hold import locked, latest_status, module


def pending_source(node, clock, joint, *, fragment=False, capture_gap=.494414091,
                   start_age=.530324221):
    """Place a real synthetic RGB-D sample at measured gen24-style clock gaps.

    Only source transport and clocks are synthetic. Production segmentation,
    RGB refinement, fragment recovery and tracker decisions remain real.
    """
    reference = node._locked_rgb_hold_reference
    ref_ns = reference['source_stamp_ns']
    source_ns = ref_ns + round(capture_gap * 1e9)
    clock[0] = ref_ns / 1e9 + start_age
    rgb = np.full((240, 320, 3), (60, 90, 190), np.uint8)
    rgb[106:134, 148:173] = 20
    depth = np.full((240, 320), .4, np.float32)
    if fragment:
        depth[106:134, 148:173] = 0.
        depth[108:116, 151:169] = .35
        depth[124:132, 151:169] = .35
    else:
        depth[106:134, 148:173] = .35
    pair = (node.bridge.cv2_to_imgmsg(rgb, 'bgr8'),
            node.bridge.cv2_to_imgmsg(depth, '32FC1'))
    for msg in pair:
        msg.header.stamp = module.rospy.Time(source_ns // 10**9, source_ns % 10**9)
        msg.header.frame_id = node.camera_frame
    joint(source_ns / 1e9 - .025)
    joint(source_ns / 1e9 + .025)
    node.latest = pair
    return pair, reference


def snapshot(node):
    return (node.tracker.last, node._locked_rgb_hold_reference,
            node.last_good_source_stamp_ns, node.tracker.anchor)


def assert_not_committed(node, before):
    last, reference, last_good, anchor = before
    assert node.tracker.last is last
    assert node.tracker.anchor is anchor
    assert node.last_good_source_stamp_ns == last_good
    assert node._locked_rgb_hold_reference is None or node._locked_rgb_hold_reference is reference
    assert not node.pubs['object'].publish.call_args[0][0].detected
    assert not np.any(node.bridge.imgmsg_to_cv2(
        node.pubs['object_mask'].publish.call_args[0][0], 'mono8'))
    assert getattr(node, '_arrival_frame_completion', None) is None


@pytest.mark.parametrize('path', ['raw_match', 'fragment'])
@pytest.mark.parametrize('failure_at', ['rgb_post', 'commit'])
def test_both_staged_paths_reject_expiry_without_mutating_live_references(
        monkeypatch, path, failure_at):
    node, clock, _, joint, _, _ = completed(monkeypatch)
    pair, reference = pending_source(node, clock, joint, fragment=path == 'fragment')
    before = snapshot(node)
    calls = []
    operation_name = ('recover_arrival_fragment_group' if path == 'fragment'
                      else 'refine_candidate')
    original = getattr(module, operation_name)

    def measured_work(*args, **kwargs):
        result = original(*args, **kwargs)
        calls.append(operation_name)
        assert node.tracker.last is before[0]
        if path == 'fragment':
            assert result[1] is not None
        assert getattr(node, '_arrival_frame_completion', None) is not None
        # Reproduce the old reference crossing .8 while CURRENT remains fresh.
        clock[0] = reference['source_stamp_ns'] / 1e9 + .828624487
        if failure_at == 'rgb_post':
            clock[0] = pair[0].header.stamp.to_sec() + .801
        return result

    monkeypatch.setattr(module, operation_name, measured_work)
    commit_hits = []
    if failure_at == 'commit':
        prepared_hold = [False]
        armed = [False]
        prepare = node._prepare_locked_rgb_hold_reference
        check = node._observation_arrival_retry_reference
        original_lock = node.input_lock

        def prepared(*args, **kwargs):
            result = prepare(*args, **kwargs)
            assert result is not None
            prepared_hold[0] = True
            return result

        def after_post_check(*args, **kwargs):
            result = check(*args, **kwargs)
            if prepared_hold[0] and kwargs.get('check_last') is False:
                assert result is reference
                armed[0] = True
            return result

        class CommitWait:
            def __enter__(self):
                original_lock.acquire()
                if armed[0]:
                    armed[0] = False
                    commit_hits.append(True)
                    clock[0] = pair[0].header.stamp.to_sec() + .801
                return self

            def __exit__(self, *args):
                original_lock.release()

        node.input_lock = CommitWait()
        monkeypatch.setattr(node, '_prepare_locked_rgb_hold_reference', prepared)
        monkeypatch.setattr(node, '_observation_arrival_retry_reference', after_post_check)
    node.process(None)
    assert calls == [operation_name]
    assert commit_hits == ([True] if failure_at == 'commit' else [])
    assert_not_committed(node, before)


@pytest.mark.parametrize('entry', ['late_capture', 'late_start'])
def test_late_entry_never_acquires_completion_for_either_deadline(monkeypatch, entry):
    node, clock, _, joint, _, _ = completed(monkeypatch)
    kwargs = (dict(capture_gap=.81, start_age=.84) if entry == 'late_capture'
              else dict(capture_gap=.70, start_age=.801))
    pending_source(node, clock, joint, **kwargs)
    observed_tokens = []
    refinement_modes = []
    original = module.segment_with_support_fallback
    refine = module.refine_candidate

    def observe(*args, **kwargs):
        observed_tokens.append(getattr(node, '_arrival_frame_completion', None))
        return original(*args, **kwargs)

    def original_refinement(*args, **kwargs):
        refinement_modes.append(kwargs.get('observation_arrival_retry', False))
        return refine(*args, **kwargs)

    monkeypatch.setattr(module, 'segment_with_support_fallback', observe)
    monkeypatch.setattr(module, 'refine_candidate', original_refinement)
    node.process(None)
    assert not any(token is not None for token in observed_tokens)
    assert not any(refinement_modes)
    assert getattr(node, '_arrival_frame_completion', None) is None


def active_token(monkeypatch):
    node, clock, _, joint, params, marker = completed(monkeypatch)
    pair, reference = pending_source(node, clock, joint)
    assert node._observation_arrival_retry_reference(pair, params['/camera']) is reference
    node._processing_timing = dict(processing_start_ros_ns=module.rospy.Time.now().to_nsec(),
                                  source_stamp_ns=pair[0].header.stamp.to_nsec())
    node._begin_arrival_frame_completion(pair, reference)
    token = getattr(node, '_arrival_frame_completion', None)
    assert token is not None
    clock[0] = reference['source_stamp_ns'] / 1e9 + .828624487
    assert node._observation_arrival_retry_reference(pair, params['/camera'], expected=reference) is reference
    return node, clock, joint, params, marker, pair, reference, token


@pytest.mark.parametrize('change', [
    'source', 'reference', 'marker', 'lease', 'current_expired', 'session_expired',
    'monotonic_expired', 'manual', 'out_and_back', 'support_pending',
    'metric_pending', 'retired'])
def test_one_frame_completion_cannot_be_borrowed_or_survive_new_invalidity(monkeypatch, change):
    node, clock, joint, params, marker, pair, reference, token = active_token(monkeypatch)
    if change == 'source':
        pair = deepcopy(pair)
        for msg in pair:
            msg.header.stamp += module.rospy.Duration(.001)
    elif change == 'reference':
        node._locked_rgb_hold_reference = dict(reference)
    elif change == 'marker':
        reference['observation_arrival_retry'] = dict(marker)
    elif change == 'lease':
        marker['lease_id'] = 'another_lease'
    elif change == 'current_expired':
        clock[0] = pair[0].header.stamp.to_sec() + .801
    elif change == 'session_expired':
        marker['session_deadline_ns'] = module.rospy.Time.now().to_nsec()
    elif change == 'monotonic_expired':
        clock[0] = token['completion_monotonic'] + .001
    elif change == 'manual':
        params['/gui/joint_direct_mode'] = True
    elif change == 'out_and_back':
        joint(clock[0], [.005, 0., 0., 0., 0., 0.])
        joint(clock[0] + .001, [0.] * 6)
    elif change == 'support_pending':
        node._locked_source_pending = dict(kind='support_measurement_gap')
    elif change == 'metric_pending':
        node._metric_ambiguity_pending = dict(kind='ambiguous')
    else:
        node.tracker.lost = True
    assert node._observation_arrival_retry_reference(
        pair, params['/camera'], expected=reference) is None
    assert node.last_good_source_stamp_ns == reference['source_stamp_ns']


@pytest.mark.parametrize('scope', ['support_pending', 'metric_pending', 'initial', 'ordinary', 'retired'])
def test_non_normal_arrival_scope_does_not_receive_completion_token(monkeypatch, scope):
    if scope in ('initial', 'ordinary'):
        node, clock, _, joint, params = locked(monkeypatch, initial_station=scope == 'initial')
    else:
        node, clock, _, joint, params, _ = completed(monkeypatch)
    pair, reference = pending_source(node, clock, joint)
    if scope == 'support_pending':
        node._locked_source_pending = dict(kind='support_measurement_gap')
    elif scope == 'metric_pending':
        node._metric_ambiguity_pending = dict(kind='ambiguous')
    elif scope == 'retired':
        node.tracker.lost = True
    admitted_reference = node._observation_arrival_retry_reference(pair, params['/camera'])
    node._begin_arrival_frame_completion(pair, admitted_reference)
    assert getattr(node, '_arrival_frame_completion', None) is None
