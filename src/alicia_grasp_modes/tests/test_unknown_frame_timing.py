"""Frame-age diagnosis is numeric evidence, never new frame authority."""
import json
import math
from copy import deepcopy
from types import MethodType
from unittest.mock import Mock

import pytest

from test_unknown_tf_alignment import prepared, module


def with_status(monkeypatch, **kwargs):
    n, clock, calls, segmentation, raw = prepared(monkeypatch, **kwargs)
    n.status = MethodType(module.UnknownTabletopPerception.status, n)
    n.pubs['detector_status'] = Mock()
    raw.metrics = {}
    pair = n.latest
    pair[0].header.seq, pair[1].header.seq = 471, 481
    n.latest = None
    n.frame(*pair)
    return n, clock, calls, segmentation, raw


def status(n):
    return json.loads(n.pubs['detector_status'].publish.call_args[0][0].data)


@pytest.mark.parametrize('stage', ['input_age_check', 'before_tf', 'post_tf', 'post_segment', 'post_refine'])
def test_each_stale_checkpoint_retains_original_loss_and_identifies_failed_stage(monkeypatch, stage):
    n, clock, calls, segment, raw = with_status(monkeypatch,
        age=.81 if stage == 'input_age_check' else .02,
        delay=.79 if stage == 'post_tf' else .28)
    if stage == 'before_tf':
        get_param = module.rospy.get_param
        def slow_param(*args):
            value = get_param(*args)
            clock[0] += .79
            return value
        monkeypatch.setattr(module.rospy, 'get_param', slow_param)
    if stage == 'post_segment':
        segment.side_effect = lambda *a, **kw: (clock.__setitem__(0, clock[0]+.51), object())[1]
    if stage == 'post_refine':
        def refine(*_):
            clock[0] += .51
            return raw
        monkeypatch.setattr(module, 'refine_candidate', refine)
    n.process(None)
    result = status(n)
    timing = result['frame_timing']
    assert timing['failed_stage'] == stage
    assert timing['stage'] == 'failure_publish'
    assert timing['failure_source_age_sec'] > .8
    assert timing['source_stamp_ns'] == 10_000_000_000
    assert timing['max_frame_age_sec'] == .8
    assert timing['status_source_age_sec'] > .8
    assert timing['processing_start_source_age_sec'] == pytest.approx(.81 if stage == 'input_age_check' else .02)
    assert timing['ingress_source_age_sec'] == pytest.approx(.81 if stage == 'input_age_check' else .02)
    assert timing['ingress_to_processing_sec'] >= 0.
    assert timing['latest_pending_source_stamp_ns'] is None
    assert timing['color_seq'] == result['color_seq'] == 471
    assert timing['depth_seq'] == result['depth_seq'] == 481
    assert n.tracker.lost and not n.pubs['object'].publish.call_args[0][0].detected
    assert result['first_loss']['reason'] == 'perception_failure: source_frame_stale_or_future'
    n.pubs['object_pose_base'].publish.assert_not_called()
    if stage in ('input_age_check', 'before_tf'):
        assert not calls
    if stage in ('input_age_check', 'before_tf', 'post_tf'):
        segment.assert_not_called()
    for key in ('stage_wall_sec', 'stage_thread_cpu_sec', 'stage_end_elapsed_sec'):
        assert timing[key]
        assert all(type(v) is float and math.isfinite(v) and v >= 0. for v in timing[key].values())
    json.dumps(result, allow_nan=False)


def test_ready_status_preserves_original_ingress_sequence_even_if_publish_mutates_shared_header(monkeypatch):
    n, _, _, _, _ = with_status(monkeypatch)
    n.pubs['object_mask'].publish.side_effect = lambda msg: setattr(msg.header, 'seq', 999)
    n.process(None)
    result = status(n)
    assert result['state'] == 'ready' and n.first_loss is None
    assert result['color_seq'] == 471 and result['depth_seq'] == 481
    assert result['frame_timing']['failed_stage'] is None
    assert result['frame_timing']['stage'] == 'publish'
    assert result['source_stamp_ns'] == 10_000_000_000
    assert n.pubs['object_mask'].publish.call_args[0][0].header.stamp.to_nsec() == 10_000_000_000
    assert n.pubs['object'].publish.call_args[0][0].detected


def test_newest_pending_pair_is_reported_without_consuming_or_copying_images(monkeypatch):
    n, clock, _, _, raw = with_status(monkeypatch)
    pending = deepcopy(n.latest)
    for msg in pending:
        msg.header.stamp = module.rospy.Time.from_sec(10.5)
    def refine(*_):
        clock[0] = 10.6
        n.frame(*pending)
        clock[0] = 10.7
        return raw
    monkeypatch.setattr(module, 'refine_candidate', refine)
    n.process(None)
    timing = status(n)['frame_timing']
    assert timing['latest_pending_source_stamp_ns'] == 10_500_000_000
    assert timing['latest_pending_source_age_sec'] == pytest.approx(.2)
    assert timing['latest_pending_residence_sec'] >= 0.
    assert n.latest[0] is pending[0] and n.latest[1] is pending[1]
    assert n.last_consumed_ns == 10_000_000_000
    assert timing['source_stamp_ns'] == 10_000_000_000


def test_selection_change_during_segmentation_discards_old_timing_without_affecting_reset(monkeypatch):
    n, _, _, segment, _ = with_status(monkeypatch)
    def newer_selection(*_a, **_kw):
        n.generation += 1
        n._processing_timing = None
        n._latest_frame_timing = None
        return object()
    segment.side_effect = newer_selection
    n.process(None)
    n.pubs['object'].publish.assert_not_called()
    n.pubs['detector_status'].publish.assert_not_called()
    assert n._processing_timing is None
