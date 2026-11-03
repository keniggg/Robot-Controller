"""Compound RGB timing must observe existing work without acquiring authority."""
from contextlib import contextmanager
from copy import deepcopy
import json
import threading

import numpy as np
import pytest

from test_unknown_frame_timing import with_status
from test_locked_rgb_hold import locked, latest_status, module


def component_fixture(monkeypatch):
    n, *_ = with_status(monkeypatch, delay=0.)
    clock = [100., 20., 30.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(module.time, 'thread_time', lambda: clock[1])
    monkeypatch.setattr(module.time, 'process_time', lambda: clock[2])
    n._begin_frame_timing(n.latest, None, clock[0])
    n._frame_timing_stage('rgb_refinement')
    return n, clock


def test_repeated_components_record_cpu_and_original_exception_without_changing_parent(monkeypatch):
    n, clock = component_fixture(monkeypatch)
    # Minimal older diagnostics also retain the original exception.
    n._processing_timing.pop('rgb_components')
    with n._rgb_timing_component('hold.context_initial'):
        clock[:] = [100.2, 20.01, 30.04]
    error = ValueError('original_context_failure')
    with pytest.raises(ValueError) as raised:
        with n._rgb_timing_component('hold.context_initial'):
            clock[:] = [100.5, 20.03, 30.10]
            raise error
    assert raised.value is error
    timing = n._source_timing_status(n.latest)
    rows = timing['rgb_components']['hold.context_initial']
    assert [r['completed'] for r in rows] == [True, False]
    assert [r['wall_sec'] for r in rows] == pytest.approx([.2, .3])
    assert [r['thread_cpu_sec'] for r in rows] == pytest.approx([.01, .02])
    assert [r['process_cpu_sec'] for r in rows] == pytest.approx([.04, .06])
    assert timing['stage'] == 'rgb_refinement'
    assert timing['stage_wall_sec']['rgb_refinement'] == pytest.approx(.5)
    rows[0]['wall_sec'] = 999.
    assert n._processing_timing['rgb_components']['hold.context_initial'][0]['wall_sec'] == pytest.approx(.2)
    json.dumps(timing, allow_nan=False)


@pytest.mark.parametrize('replacement', [None, {}])
def test_selection_reset_or_replacement_does_not_receive_inflight_components(monkeypatch, replacement):
    n, _ = component_fixture(monkeypatch)
    old = n._processing_timing
    with n._rgb_timing_component('refine'):
        n._processing_timing = replacement
    assert old['rgb_components'] == {}
    assert replacement is None or replacement == {}
    n._processing_timing = None
    with pytest.raises(ValueError, match='unchanged'):
        with n._rgb_timing_component('refine'):
            raise ValueError('unchanged')


def test_other_callback_thread_cannot_attach_components_to_processing_frame(monkeypatch):
    n, _ = component_fixture(monkeypatch)
    errors = []
    def callback():
        try:
            with n._rgb_timing_component('context.source_tf'):
                raise ValueError('callback_error')
        except ValueError as exc:
            errors.append(str(exc))
    worker = threading.Thread(target=callback)
    worker.start(); worker.join(timeout=1.)
    assert not worker.is_alive() and errors == ['callback_error']
    assert n._processing_timing['rgb_components'] == {}


def test_input_lock_duration_excludes_held_work_and_releases_on_original_error(monkeypatch):
    n, clock = component_fixture(monkeypatch)
    released = []
    class Lock:
        def __enter__(self):
            clock[0] += .15
        def __exit__(self, *exc):
            released.append(exc[0])
    n.input_lock = Lock()
    with pytest.raises(ValueError, match='held_work_error'):
        with n._rgb_timed_input_lock('hold.input_lock_wait'):
            clock[0] += .3
            raise ValueError('held_work_error')
    row = n._processing_timing['rgb_components']['hold.input_lock_wait'][0]
    assert row['wall_sec'] == pytest.approx(.15) and row['completed'] is True
    assert released == [ValueError]


def test_real_normal_producer_keeps_mask_geometry_and_hold_with_timing_on(monkeypatch):
    n, clock, feed, _, _ = locked(monkeypatch)
    original_timing = n._rgb_timing_component
    @contextmanager
    def disabled(_name):
        yield lambda: None
    n._rgb_timing_component = disabled
    feed(target_depth=.35)
    before_mask = n.bridge.imgmsg_to_cv2(n.pubs['object_mask'].publish.call_args[0][0]).copy()
    before_object = deepcopy(n.pubs['object'].publish.call_args[0][0])
    before_hold = n._locked_rgb_hold_reference
    n._rgb_timing_component = original_timing
    clock[0] += .2
    pair = feed(target_depth=.35)
    after_object = n.pubs['object'].publish.call_args[0][0]
    after_hold = n._locked_rgb_hold_reference
    np.testing.assert_array_equal(before_mask, n.bridge.imgmsg_to_cv2(n.pubs['object_mask'].publish.call_args[0][0]))
    assert before_object.pose_base.pose == after_object.pose_base.pose
    assert after_object.detected and not n.tracker.lost and n.first_loss is None
    for key in ('bgr', 'mask', 'histogram', 'contour', 'q_source', 'reference_plane', 'reference_footprint'):
        np.testing.assert_array_equal(before_hold[key], after_hold[key])
        assert not after_hold[key].flags.writeable
    assert after_hold['source_stamp_ns'] == pair[0].header.stamp.to_nsec() > before_hold['source_stamp_ns']
    timing = latest_status(n)['frame_timing']
    components = timing['rgb_components']
    for key in ('refine', 'hold.total', 'hold.context_initial', 'hold.context_recheck',
                'hold.feedback', 'hold.bridge_conversion', 'hold.mask_rgb_validity',
                'hold.appearance', 'hold.hash_copy_fingerprints', 'hold.input_lock_wait'):
        assert len(components[key]) == 1 and components[key][0]['completed'] is True
    # Two existing context lookups remain two: no extra TF/cached parameter reads.
    assert len(components['context.source_tf']) == len(components['context.latest_tf']) == 2
    assert timing['failed_stage'] is None and timing['max_frame_age_sec'] == .8
    assert n._processing_timing_thread_ident is None
