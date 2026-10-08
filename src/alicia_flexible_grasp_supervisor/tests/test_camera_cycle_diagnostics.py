"""Offline fault injection for camera publication stalls; no ROS node/device."""
from types import SimpleNamespace

import pytest

from test_camera_node_recovery import (
    FakeBridge, FakeRospy, SinglePairCamera, load_camera_node,
)


def camera(monkeypatch):
    module = load_camera_node()
    ros = FakeRospy({'fallback_to_simulation': False})
    module.rospy, module.CvBridge = ros, FakeBridge
    module.RealSenseManager = SinglePairCamera
    node = module.CameraNode()
    clock = SimpleNamespace(now=100.)
    node._diagnostic_clock = lambda: clock.now
    node.cfg['mode_depth_preset'] = {'enabled': True}
    monkeypatch.setattr(ros, 'get_param_cached', lambda *args: {'mode': 'unknown'}, raising=False)
    node.cam.set_depth_preset_mode = lambda mode: False
    ros.shutdown_after_stamp = True
    return node, ros, clock


@pytest.mark.parametrize('slow_stage', ['sync_mode', 'read', 'runtime_params', 'image_publish'])
def test_reports_injected_stage_after_unchanged_rgb_depth_publication(monkeypatch, slow_stage):
    node, ros, clock = camera(monkeypatch)
    events = []
    def mode(_):
        events.append('mode')
        if slow_stage == 'sync_mode':
            clock.now += .4
        return False
    node.cam.set_depth_preset_mode = mode
    original_read = node.cam.read
    def read():
        events.append('read')
        if slow_stage == 'read':
            clock.now += .4
        return original_read()
    node.cam.read = read
    # Exercise the post-read stationary-temporal metadata update using the
    # real namespace publication method, rather than bypassing that branch.
    node.cam.runtime_profile['stationary_temporal'] = {'active': True}
    original_params = node._publish_runtime_camera_params
    def params():
        events.append('params')
        if slow_stage == 'runtime_params':
            clock.now += .4
        original_params()
    node._publish_runtime_camera_params = params
    original_publish = node.publish_image
    def publish(pub, array, encoding, stamp):
        events.append(encoding)
        if slow_stage == 'image_publish' and encoding == 'bgr8':
            clock.now += .4
        original_publish(pub, array, encoding, stamp)
    node.publish_image = publish
    original_warn = ros.logwarn
    def warn(*args):
        events.append('diagnostic')
        original_warn(*args)
    ros.logwarn = warn

    node.spin()

    assert events == ['mode', 'read', 'params', 'bgr8', '16UC1', 'diagnostic']
    assert len(ros.warnings) == 1
    _, total, sync_mode, reading, runtime, publishing, result = ros.warnings[0]
    assert total == pytest.approx(.4)
    segments = dict(sync_mode=sync_mode, read=reading, runtime_params=runtime,
                    image_publish=publishing)
    assert segments.pop(slow_stage) == pytest.approx(.4)
    assert all(value == pytest.approx(0.) for value in segments.values())
    assert result == 'published'
    assert node.pub_color.messages[0].header.stamp == node.pub_depth.messages[0].header.stamp
    assert ros.time_calls == 1
    assert ros.sleep_count == 0
    assert node._read_failures == 0


def test_mode_change_runtime_publication_is_not_counted_as_mode_delay(monkeypatch):
    node, ros, clock = camera(monkeypatch)
    events = []
    def mode(_):
        events.append('mode')
        clock.now += .025
        return True
    node.cam.set_depth_preset_mode = mode
    original = node._publish_runtime_camera_params
    def params():
        events.append('params')
        clock.now += .31
        original()
    node._publish_runtime_camera_params = params
    read = node.cam.read
    def next_frame():
        events.append('read')
        return read()
    node.cam.read = next_frame

    node.spin()

    assert events == ['mode', 'params', 'read']
    _, total, sync_mode, reading, runtime, publishing, result = ros.warnings[0]
    assert total == pytest.approx(.335)
    assert sync_mode == pytest.approx(.025)
    assert runtime == pytest.approx(.31)
    assert reading == publishing == 0.
    assert result == 'published'


def test_normal_cycle_adds_no_warning(monkeypatch):
    node, ros, clock = camera(monkeypatch)
    read = node.cam.read
    def next_frame():
        clock.now += .04
        return read()
    node.cam.read = next_frame
    node.spin()
    assert not ros.warnings
    assert len(node.pub_color.messages) == len(node.pub_depth.messages) == 1


def test_slow_read_exception_remains_original_recovery_path(monkeypatch):
    node, ros, clock = camera(monkeypatch)
    failure = RuntimeError('injected read timeout')
    def fail():
        clock.now += .5
        raise failure
    node.cam.read = fail
    recovered = []
    def recover(exc):
        recovered.append(exc)
        ros.shutdown = True
        return True
    node._recover_from_read_error = recover
    node.spin()
    assert recovered == [failure]
    assert len(ros.warnings) == 1
    assert ros.warnings[0][3] == pytest.approx(.5)
    assert ros.warnings[0][-1] == 'read_or_sync_error'
    assert not node.pub_color.messages and not node.pub_depth.messages


def test_throttled_publication_still_reports_slow_read(monkeypatch):
    node, ros, clock = camera(monkeypatch)
    read = node.cam.read
    def next_frame():
        clock.now += .4
        ros.shutdown = True
        return read()
    node.cam.read = next_frame
    node._publication_is_due = lambda: False
    node.spin()
    assert ros.warnings[0][3] == pytest.approx(.4)
    assert ros.warnings[0][-1] == 'publication_not_due'
    assert ros.time_calls == 0
    assert not node.pub_color.messages and not node.pub_depth.messages


def test_slow_read_reports_only_current_backend_breakdown(monkeypatch):
    import json
    node, ros, clock = camera(monkeypatch)
    original = node.cam.read
    details = dict(schema=1, result='complete', failed_stage=None,
                   total_sec=.4, stages_sec={'wait_for_frames': .4})
    def read():
        clock.now += .4
        node.cam.last_read_diagnostics = details
        return original()
    node.cam.read = read
    node.spin()
    assert len(ros.warnings) == 2
    assert ros.warnings[0][-1] == 'published'
    assert ros.warnings[1][0] == 'Camera backend read stages: %s'
    assert json.loads(ros.warnings[1][1]) == details
    assert ros.time_calls == 1


def test_sync_failure_does_not_relabel_prior_read_as_current(monkeypatch):
    node, ros, clock = camera(monkeypatch)
    node.cam.last_read_diagnostics = {'stages_sec': {'wait_for_frames': 5.}}
    node._report_slow_cycle(clock.now - .3, {'sync_mode': .3}, 'read_or_sync_error')
    assert len(ros.warnings) == 1


def test_fast_cycle_does_not_log_backend_diagnostics(monkeypatch):
    node, ros, clock = camera(monkeypatch)
    node.cam.last_read_diagnostics = {'stages_sec': {'wait_for_frames': .02}}
    node._report_slow_cycle(clock.now - .02, {'read': .02}, 'published')
    assert ros.warnings == []


def test_malformed_optional_backend_diagnostic_cannot_disrupt_publication(monkeypatch):
    node, ros, clock = camera(monkeypatch)
    original = node.cam.read
    def read():
        clock.now += .4
        node.cam.last_read_diagnostics = {'bad_metadata': object()}
        return original()
    node.cam.read = read
    node.spin()
    assert len(node.pub_color.messages) == len(node.pub_depth.messages) == 1
    assert node._read_failures == 0
    assert len(ros.warnings) == 1


def test_raw_and_previous_metadata_use_existing_slow_log_only(monkeypatch):
    import json
    node, ros, clock = camera(monkeypatch)
    current = {'color': {'frame_number': 102, 'timestamp_domain': 'hardware_clock'}}
    previous = {'color': {'frame_number': 101, 'timestamp_domain': 'hardware_clock'}}
    node.cam.last_read_diagnostics = dict(schema=2, raw_frames=current,
        previous_completed_raw_frames={'raw_frames': previous})
    node._report_slow_cycle(clock.now - .4, {'read': .4}, 'published')
    details = json.loads(ros.warnings[1][1])
    assert details['raw_frames'] == current
    assert details['previous_completed_raw_frames']['raw_frames'] == previous
    assert len(ros.warnings) == 2
    assert ros.time_calls == 0


def test_repeated_pair_rejection_never_publishes_or_triggers_camera_restart(monkeypatch):
    import json
    node, ros, clock = camera(monkeypatch)
    good_read = node.cam.read
    reads = []
    recoveries = []
    node._recover_from_read_error = lambda exc: recoveries.append(exc)
    def read():
        reads.append(True)
        if len(reads) <= 4:  # More than the configured read-failure limit.
            node.cam.last_read_diagnostics = dict(result='rejected_pair',
                pair_timing={'accepted': False, 'reason': 'timestamp_skew_exceeded',
                             'skew_ms': 436.794, 'max_skew_ms': 1000. / 60.},
                raw_frames={'color': {'device_timestamp_ms': 1790690987815.4678},
                            'depth': {'device_timestamp_ms': 1790690988252.2615}})
            return None, None
        return good_read()
    node.cam.read = read
    node.spin()
    assert len(reads) == 5 and not recoveries
    assert ros.time_calls == 1  # Only the subsequent valid pair gets a stamp.
    assert len(node.pub_color.messages) == len(node.pub_depth.messages) == 1
    assert node._read_failures == 0 and ros.sleep_count == 0
    assert len(ros.warnings) == 4
    assert json.loads(ros.warnings[0][2])['pair_timing']['reason'] == 'timestamp_skew_exceeded'


@pytest.mark.parametrize('missing', ['color', 'depth', 'both'])
def test_partial_or_unverifiable_pair_never_gets_a_common_stamp(monkeypatch, missing):
    node, ros, clock = camera(monkeypatch)
    def read():
        ros.shutdown = True
        node.cam.last_read_diagnostics = {'result': 'missing_frame'}
        color, depth = SinglePairCamera.read(node.cam)
        return (None if missing in ('color', 'both') else color,
                None if missing in ('depth', 'both') else depth)
    node.cam.read = read
    node.spin()
    assert ros.time_calls == 0
    assert not node.pub_color.messages and not node.pub_depth.messages
    assert node._read_failures == 0


def test_malformed_rejection_diagnostic_cannot_trigger_read_recovery(monkeypatch):
    node, ros, clock = camera(monkeypatch)
    def read():
        ros.shutdown = True
        node.cam.last_read_diagnostics = {'nonserializable': object()}
        return None, None
    node.cam.read = read
    node.spin()
    assert ros.time_calls == 0 and node._read_failures == 0
    assert 'diagnostic_serialization' in ros.warnings[0][2]
