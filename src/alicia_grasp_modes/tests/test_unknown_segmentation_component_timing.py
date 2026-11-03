"""Segmentation diagnostics preserve the original work, gates and exceptions."""
from contextlib import contextmanager
from copy import deepcopy
import json
import threading

import numpy as np
import pytest

from test_unknown_rgb_component_timing import component_fixture
from test_unknown_frame_timing import with_status, status
from test_locked_rgb_hold import locked, latest_status, module


def setup_component(monkeypatch):
    node, clock = component_fixture(monkeypatch)
    node._frame_timing_stage('segmentation')
    counts = [dict(collections=10+i, collected=20+i, uncollectable=i) for i in range(3)]
    monkeypatch.setattr(module.gc, 'get_stats', lambda: deepcopy(counts))
    return node, clock, counts


def test_component_records_distinct_cpu_and_gc_counts_without_changing_parent_or_error(monkeypatch):
    node, clock, counts = setup_component(monkeypatch)
    with node._segmentation_timing_component('segment_with_support_fallback'):
        clock[:] = [100.2, 20.03, 30.05]
        counts[0]['collections'] += 2
        counts[1]['collected'] += 7
    error = ValueError('original_segment_failure')
    with pytest.raises(ValueError) as caught:
        with node._segmentation_timing_component('segment_with_support_fallback'):
            clock[:] = [100.5, 20.04, 30.12]
            counts[2]['uncollectable'] += 1
            raise error
    assert caught.value is error
    snapshot = node._source_timing_status(node.latest)
    rows = snapshot['segmentation_components']['segment_with_support_fallback']
    assert [r['completed'] for r in rows] == [True, False]
    assert [r['wall_sec'] for r in rows] == pytest.approx([.2, .3])
    assert [r['thread_cpu_sec'] for r in rows] == pytest.approx([.03, .01])
    assert [r['process_cpu_sec'] for r in rows] == pytest.approx([.05, .07])
    assert rows[0]['gc_generation_deltas'] == [
        dict(generation=0, collections=2, collected=0, uncollectable=0),
        dict(generation=1, collections=0, collected=7, uncollectable=0),
        dict(generation=2, collections=0, collected=0, uncollectable=0)]
    assert rows[1]['gc_generation_deltas'][2]['uncollectable'] == 1
    assert rows[0]['gc_counts_scope'] == 'process_wide_counts_not_duration_or_thread_attribution'
    assert snapshot['stage'] == 'segmentation'
    assert snapshot['stage_wall_sec']['segmentation'] == pytest.approx(.5)
    rows[0]['gc_generation_deltas'][0]['collections'] = 999
    assert node._processing_timing['segmentation_components']['segment_with_support_fallback'][0]['gc_generation_deltas'][0]['collections'] == 2
    json.dumps(snapshot, allow_nan=False)


@pytest.mark.parametrize('replacement', [None, {}])
def test_reset_and_replacement_do_not_receive_inflight_segmentation_diagnostics(monkeypatch, replacement):
    node, _, _ = setup_component(monkeypatch)
    old = node._processing_timing
    with node._segmentation_timing_component('station_qualification'):
        node._processing_timing = replacement
    assert old['segmentation_components'] == {}
    assert replacement is None or replacement == {}
    node._processing_timing = None
    with pytest.raises(ValueError, match='unchanged'):
        with node._segmentation_timing_component('station_qualification'):
            raise ValueError('unchanged')


def test_other_callback_thread_cannot_attach_counts_to_processing_frame(monkeypatch):
    node, _, _ = setup_component(monkeypatch)
    calls = []
    monkeypatch.setattr(module.gc, 'get_stats', lambda: calls.append(True))
    errors = []
    def callback():
        try:
            with node._segmentation_timing_component('station_qualification'):
                raise ValueError('callback_only')
        except ValueError as exc:
            errors.append(str(exc))
    thread = threading.Thread(target=callback)
    thread.start(); thread.join(timeout=1.)
    assert not thread.is_alive() and errors == ['callback_only']
    assert not calls and not node._processing_timing['segmentation_components']


def test_lock_wait_excludes_work_inside_and_preserves_release_on_error(monkeypatch):
    node, clock, counts = setup_component(monkeypatch)
    released = []
    class Lock:
        def __enter__(self):
            clock[0] += .15
            counts[0]['collections'] += 1
        def __exit__(self, *exc):
            released.append(exc[0])
    node.lock = Lock()
    with pytest.raises(ValueError, match='held_work_failure'):
        with node._segmentation_timed_lock():
            clock[0] += .3
            counts[0]['collections'] += 10
            raise ValueError('held_work_failure')
    row = node._processing_timing['segmentation_components']['station_lock_wait'][0]
    assert row['wall_sec'] == pytest.approx(.15)
    assert row['gc_generation_deltas'][0]['collections'] == 1
    assert row['completed'] is True and released == [ValueError]


@pytest.mark.parametrize('failed_component', ['station_qualification', 'segment_with_support_fallback'])
def test_real_process_failure_is_preserved_and_later_work_does_not_run(monkeypatch, failed_component):
    node, _, _, segment, _ = with_status(monkeypatch, delay=0.)
    if failed_component == 'station_qualification':
        def reject(*args, **kwargs):
            raise ValueError('original_station_rejection')
        monkeypatch.setattr(node, '_observation_arrival_retry_reference', reject)
        reason = 'original_station_rejection'
    else:
        segment.side_effect = ValueError('original_segmentation_rejection')
        reason = 'original_segmentation_rejection'
    node.process(None)
    value = status(node)
    assert value['frame_timing']['failed_stage'] == 'segmentation'
    assert value['first_loss']['reason'] == 'perception_failure: ' + reason
    assert value['frame_timing']['segmentation_components'][failed_component][0]['completed'] is False
    assert not node.pubs['object'].publish.call_args[0][0].detected
    node.pubs['object_pose_base'].publish.assert_not_called()
    if failed_component == 'station_qualification':
        segment.assert_not_called()


def test_real_normal_producer_same_geometry_and_original_segmentation_calls_with_diagnostics(monkeypatch):
    node, clock, feed, _, _ = locked(monkeypatch)
    observed = []
    original_segment = module.segment_with_support_fallback
    def segment(*args, **kwargs):
        observed.append((args, kwargs))
        return original_segment(*args, **kwargs)
    monkeypatch.setattr(module, 'segment_with_support_fallback', segment)
    original_timing = node._segmentation_timing_component
    @contextmanager
    def disabled(_name):
        yield lambda: None
    node._segmentation_timing_component = disabled
    feed(target_depth=.35)
    before_mask = node.bridge.imgmsg_to_cv2(node.pubs['object_mask'].publish.call_args[0][0]).copy()
    before_object = deepcopy(node.pubs['object'].publish.call_args[0][0])
    before_reference = node._locked_rgb_hold_reference
    node._segmentation_timing_component = original_timing
    clock[0] += .2
    pair = feed(target_depth=.35)
    assert len(observed) == 2
    np.testing.assert_array_equal(observed[0][0][0], observed[1][0][0])
    assert set(observed[0][1]) == set(observed[1][1])
    np.testing.assert_array_equal(before_mask, node.bridge.imgmsg_to_cv2(node.pubs['object_mask'].publish.call_args[0][0]))
    assert before_object.pose_base.pose == node.pubs['object'].publish.call_args[0][0].pose_base.pose
    assert node.last_good_source_stamp_ns == pair[0].header.stamp.to_nsec()
    assert node._locked_rgb_hold_reference['source_stamp_ns'] > before_reference['source_stamp_ns']
    assert not node.tracker.lost and node.first_loss is None
    evidence = latest_status(node)['frame_timing']
    assert set(evidence['segmentation_components']) == {'station_lock_wait', 'station_qualification', 'segment_with_support_fallback'}
    for rows in evidence['segmentation_components'].values():
        assert len(rows) == 1 and rows[0]['completed'] is True
        assert rows[0]['stage'] == 'segmentation' and len(rows[0]['gc_generation_deltas']) == 3
    assert evidence['max_frame_age_sec'] == .8 and evidence['failed_stage'] is None
