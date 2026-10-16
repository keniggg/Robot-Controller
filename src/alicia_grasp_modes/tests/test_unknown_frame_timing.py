"""Frame-age diagnosis is numeric evidence, never new frame authority."""
import json
import math
from copy import deepcopy
from types import MethodType
from unittest.mock import Mock

import pytest
import numpy as np

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


DECODE_COMPONENTS = ('camera_parameter_read', 'rectified_source_validation',
                     'cv_bridge_conversion', 'float32_depth_scale')


def instrument_decode(monkeypatch, encoding='32FC1', failed_component=None):
    n, clock, calls, segment, raw = with_status(monkeypatch, delay=0.)
    color, old_depth = n.latest
    values = np.full((20, 20), 3000, np.uint16) if encoding != '32FC1' else np.full((20, 20), .3, np.float32)
    depth_message = n.bridge.cv2_to_imgmsg(values, encoding=encoding)
    depth_message.header = old_depth.header
    n.latest = (color, depth_message)
    wall, cpu, visited = [100.], [20.], []
    monkeypatch.setattr(module.time, 'monotonic', lambda: wall[0])
    monkeypatch.setattr(module.time, 'thread_time', lambda: cpu[0])
    delays = {name: (.01 * (index+1), .001 * (index+1))
              for index, name in enumerate(DECODE_COMPONENTS)}

    def elapsed(name):
        visited.append(name)
        wall[0] += delays[name][0]
        cpu[0] += delays[name][1]
        if name == failed_component:
            raise ValueError('decode_component_failure')

    original_param = module.rospy.get_param
    def get_param(*args):
        elapsed('camera_parameter_read')
        return dict(original_param(*args), depth_scale=.0001)
    monkeypatch.setattr(module.rospy, 'get_param', get_param)
    original_validation = module.require_rectified_source_after_activation
    def validate(*args):
        elapsed('rectified_source_validation')
        return original_validation(*args)
    monkeypatch.setattr(module, 'require_rectified_source_after_activation', validate)
    original_bridge = n.bridge.imgmsg_to_cv2
    decoded = [None]
    def convert(message, **kwargs):
        if message is not depth_message:
            return original_bridge(message, **kwargs)
        assert kwargs == {'desired_encoding': 'passthrough'}
        elapsed('cv_bridge_conversion')
        decoded[0] = original_bridge(message, **kwargs)
        return decoded[0]
    monkeypatch.setattr(n.bridge, 'imgmsg_to_cv2', convert)
    original_array = module.np.asarray
    def asarray(value, *args, **kwargs):
        if value is decoded[0] and kwargs.get('dtype') is np.float32:
            elapsed('float32_depth_scale')
        return original_array(value, *args, **kwargs)
    monkeypatch.setattr(module.np, 'asarray', asarray)
    return n, segment, delays, visited


@pytest.mark.parametrize('encoding', ['16UC1', 'mono16', '32FC1'])
def test_decode_components_separate_wall_and_cpu_without_repeating_reads_or_changing_depth(monkeypatch, encoding):
    n, segment, delays, visited = instrument_decode(monkeypatch, encoding)
    pair = n.latest
    n.process(None)
    result = status(n)
    timing = result['frame_timing']
    assert result['state'] == 'ready' and timing['failed_stage'] is None
    assert visited == list(DECODE_COMPONENTS)
    assert timing['stage'] == 'publish'
    assert timing['stage_wall_sec']['decode'] == pytest.approx(.1)
    assert timing['stage_thread_cpu_sec']['decode'] == pytest.approx(.01)
    for name, (wall, cpu) in delays.items():
        component = timing['decode_components'][name]
        assert component == dict(wall_sec=pytest.approx(wall),
                                 thread_cpu_sec=pytest.approx(cpu), completed=True)
    measured_depth = segment.call_args[0][0]
    assert measured_depth.dtype == np.float32
    np.testing.assert_allclose(measured_depth, .3)
    assert n.pubs['object'].publish.call_args[0][0].header.stamp.to_nsec() == 10_000_000_000
    # A published/queried diagnostic cannot mutate the in-flight measurements.
    snapshot = n._source_timing_status(pair)
    snapshot['decode_components']['camera_parameter_read']['wall_sec'] = 999.
    assert n._processing_timing['decode_components']['camera_parameter_read']['wall_sec'] == pytest.approx(.01)
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize('failed_component', DECODE_COMPONENTS)
def test_decode_failure_keeps_finished_components_and_original_failure_stage(monkeypatch, failed_component):
    n, segment, delays, visited = instrument_decode(monkeypatch, failed_component=failed_component)
    n.process(None)
    result = status(n)
    timing = result['frame_timing']
    index = DECODE_COMPONENTS.index(failed_component)
    assert visited == list(DECODE_COMPONENTS[:index+1])
    assert set(timing['decode_components']) == set(visited)
    assert timing['failed_stage'] == 'decode'
    assert timing['stage'] == 'failure_publish'
    assert result['first_loss']['reason'] == 'perception_failure: decode_component_failure'
    for name in visited:
        component = timing['decode_components'][name]
        assert component['completed'] is (name != failed_component)
        assert component['wall_sec'] == pytest.approx(delays[name][0])
        assert component['thread_cpu_sec'] == pytest.approx(delays[name][1])
    segment.assert_not_called()
    assert not n.pubs['object'].publish.call_args[0][0].detected
    n.pubs['object_pose_base'].publish.assert_not_called()
    json.dumps(result, allow_nan=False)


def test_decode_component_does_not_repopulate_reset_or_replacement_timing(monkeypatch):
    n, *_ = with_status(monkeypatch)
    old = {'decode_components': {}}
    replacement = {'decode_components': {}}
    n._processing_timing = old
    with n._decode_timing_component('camera_parameter_read'):
        n._processing_timing = replacement
    assert old == replacement == {'decode_components': {}}
    n._processing_timing = None
    with pytest.raises(ValueError, match='original_error'):
        with n._decode_timing_component('camera_parameter_read'):
            raise ValueError('original_error')
    assert n._processing_timing is None
