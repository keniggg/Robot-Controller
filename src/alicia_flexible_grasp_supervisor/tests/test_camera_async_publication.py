"""Event-synchronized CameraNode integration; all camera/ROS endpoints are fake."""
import copy
import threading
import time

import numpy as np
import pytest

from test_camera_node_recovery import (
    FakeBridge, FakeRospy, fake_runtime_profile, load_camera_node,
)


BASE_STAMP_NS = 1791000000000000001


def wait_for(predicate, timeout=1.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.001)
    assert predicate(), 'expected asynchronous state was not reached'


class FakeTimer:
    def __init__(self, callback):
        self.callback = callback
        self.stopped = False

    def shutdown(self):
        self.stopped = True


class FakeCamera:
    def __init__(self, width, height, simulate=False, **kwargs):
        self.width, self.height, self.simulate = int(width), int(height), bool(simulate)
        self.depth_scale = .0001
        self.runtime_profile = fake_runtime_profile(self.width, self.height)
        self.started = self.stopped = False

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def set_depth_preset_mode(self, mode):
        return False

    def read(self):
        return (np.ones((self.height, self.width, 3), np.uint8),
                np.full((self.height, self.width), 100, np.uint16))


@pytest.fixture
def camera(monkeypatch):
    module = load_camera_node()
    ros = FakeRospy({'async_publication_enabled': True, 'fallback_to_simulation': False})
    class Stamp:
        def __init__(self, secs=0, nsecs=0):
            self.secs, self.nsecs = int(secs), int(nsecs)

        def to_nsec(self):
            return self.secs * 10**9 + self.nsecs

        @staticmethod
        def now():
            value = BASE_STAMP_NS + ros.time_calls * 101
            ros.time_calls += 1
            return Stamp(*divmod(value, 10**9))
    ros.Time = Stamp
    ros.Duration = lambda value: value
    ros.Timer = lambda duration, callback: FakeTimer(callback)
    ros.get_param_cached = lambda *args: {'mode': 'unknown'}
    ros.logerr_throttle = lambda *args: ros.errors.append(args)
    module.rospy, module.CvBridge, module.RealSenseManager = ros, FakeBridge, FakeCamera
    node = module.CameraNode()
    # Both read-completion and worker age use the same deliberately fixed
    # domain. Tests exercise source preservation independently of VM timing.
    node._monotonic = lambda: 10.
    node._publish_runtime_camera_params()
    yield node, ros, module
    ros.shutdown = True
    node.shutdown()


def blocked_first_color(node):
    entered, release = threading.Event(), threading.Event()
    original = node.pub_color.publish
    calls = [0]
    def publish(message):
        calls[0] += 1
        if calls[0] == 1:
            entered.set()
            assert release.wait(3.), 'test failed to release its fake publisher'
        original(message)
    node.pub_color.publish = publish
    return entered, release


def test_blocked_publication_keeps_reader_running_and_preserves_latest_whole_pair_stamp(camera):
    node, ros, _ = camera
    entered, release = blocked_first_color(node)
    color = np.zeros((node.height, node.width, 3), np.uint8)
    depth = np.zeros((node.height, node.width), np.uint16)
    reads = [0]
    def read():
        reads[0] += 1
        if reads[0] == 2:
            assert entered.wait(1.)
        color[:] = reads[0]
        depth[:] = reads[0] * 100
        if reads[0] == 8:
            ros.shutdown = True
        return color, depth
    node.cam.read = read
    node._publication_is_due = lambda: True
    reader = threading.Thread(target=node.spin, daemon=True)
    try:
        reader.start(); assert entered.wait(1.)
        reader.join(.5)
        assert not reader.is_alive(), 'SDK reader waited for a blocked ROS publisher'
        assert reads[0] == 8 and ros.time_calls == 8
        report = node._publication_worker.summary()
        assert report['publishing'] and report['pending'] == 1
        assert report['replaced_pending'] == 6
        # Recycling the fake SDK buffers after submit must not alter either
        # stream of the in-flight or pending frozen observation.
        color[:] = 99; depth[:] = 9999
        release.set()
        wait_for(lambda: node._publication_worker.summary()['published'] == 2)
        assert [int(msg.image[0, 0, 0]) for msg in node.pub_color.messages] == [1, 8]
        assert [int(msg.image[0, 0]) for msg in node.pub_depth.messages] == [100, 800]
        expected = [BASE_STAMP_NS, BASE_STAMP_NS + 7 * 101]
        assert [msg.header.stamp.to_nsec() for msg in node.pub_color.messages] == expected
        assert [msg.header.stamp.to_nsec() for msg in node.pub_depth.messages] == expected
        assert ros.time_calls == 8  # Publisher must never generate a new stamp.
        publication = ros.params['/camera']['runtime_profile']['publication']
        assert publication['image_header_clock'] == 'host_read_completion'
        assert publication['exposure_clock_calibrated'] is False
    finally:
        release.set(); reader.join(1.)


def test_new_geometry_waits_for_old_whole_pair_before_namespace_commit(camera):
    node, ros, _ = camera
    entered, release = blocked_first_color(node)
    events = []; original_depth = node.pub_depth.publish; original_params = ros.set_param
    def depth(message):
        events.append(('depth', message.header.stamp.to_nsec()))
        original_depth(message)
    def params(name, value):
        if name == '/camera':
            events.append(('geometry', value['fx']))
        original_params(name, value)
    node.pub_depth.publish, ros.set_param = depth, params
    old_worker = node._publication_worker
    color, depth_pixels = node.cam.read()
    old_worker.submit(color, depth_pixels, 10., BASE_STAMP_NS)
    assert entered.wait(1.)
    node.cam.runtime_profile['geometry_intrinsics']['fx'] = 77.
    done, errors = threading.Event(), []
    def update():
        try:
            node._publish_runtime_camera_params()
        except Exception as error:
            errors.append(error)
        finally:
            done.set()
    updater = threading.Thread(target=update, daemon=True)
    try:
        updater.start()
        wait_for(lambda: old_worker.summary()['closing'])
        assert not done.is_set() and not node._runtime_geometry_ready
        assert ros.params['/camera']['fx'] == 4. and events == []
        release.set(); assert done.wait(1.)
        assert errors == [] and node._runtime_geometry_ready
        assert events == [('depth', BASE_STAMP_NS), ('geometry', 77.)]
        assert node._publication_worker is not old_worker
        assert not old_worker.summary()['worker_alive']
    finally:
        release.set(); updater.join(1.)


def test_blocked_old_publisher_refuses_camera_replacement_and_keeps_old_geometry(camera):
    node, ros, _ = camera
    entered, release = blocked_first_color(node)
    old_camera, old_worker = node.cam, node._publication_worker
    old_worker.submit(*old_camera.read(), 10., BASE_STAMP_NS)
    assert entered.wait(1.)
    made = []
    original_make = node._make_camera
    node._make_camera = lambda simulate: made.append(simulate)
    old_params = copy.deepcopy(ros.params['/camera'])
    try:
        with pytest.raises(RuntimeError, match='still in flight'):
            node._start_camera(False)
        assert made == [] and node.cam is old_camera
        assert node._publication_worker is old_worker
        assert not node._runtime_geometry_ready
        assert ros.params['/camera'] == old_params
        release.set()
        wait_for(lambda: not old_worker.summary()['worker_alive'])
        node._make_camera = original_make
        node._start_camera(False)
        assert node.cam is not old_camera and node._runtime_geometry_ready
        assert node._publication_worker is not old_worker
    finally:
        release.set(); node._make_camera = original_make


def test_failed_metadata_commit_cannot_publish_until_successful_retry(camera):
    node, ros, _ = camera
    original_params = ros.set_param
    node.cam.runtime_profile['geometry_intrinsics']['fx'] = 77.
    def fail(name, value):
        if name == '/camera':
            raise RuntimeError('fake master rejected geometry')
        original_params(name, value)
    ros.set_param = fail
    with pytest.raises(RuntimeError, match='master rejected'):
        node._publish_runtime_camera_params()
    assert not node._runtime_geometry_ready and node._publication_worker is None
    assert ros.params['/camera']['fx'] == 4.
    assert node.pub_color.messages == node.pub_depth.messages == []
    ros.set_param = original_params
    reads = []
    original_read = node.cam.read
    def read():
        assert ros.params['/camera']['fx'] == 77. and node._runtime_geometry_ready
        reads.append(True); ros.shutdown = True
        return original_read()
    node.cam.read = read
    node.spin()
    wait_for(lambda: node._publication_worker.summary()['published'] == 1)
    assert reads == [True]
    assert node.pub_color.messages[0].header.stamp.to_nsec() == BASE_STAMP_NS
    assert node.pub_depth.messages[0].header.stamp.to_nsec() == BASE_STAMP_NS


def test_slow_temporal_xmlrpc_diagnostic_does_not_block_steady_sdk_read(camera):
    node, ros, _ = camera
    entered, release = threading.Event(), threading.Event(); original_params = ros.set_param
    node.cam.runtime_profile['stationary_temporal'] = {'active': True}
    def params(name, value):
        if name == '/camera/runtime_profile/stationary_temporal':
            entered.set(); assert release.wait(2.)
        original_params(name, value)
    ros.set_param = params
    diagnostic = threading.Thread(target=lambda: node._publish_temporal_status(None), daemon=True)
    reads = []
    original_read = node.cam.read
    def read():
        reads.append(True); ros.shutdown = True
        return original_read()
    node.cam.read = read
    node._publication_is_due = lambda: False
    reader = threading.Thread(target=node.spin, daemon=True)
    try:
        diagnostic.start(); assert entered.wait(1.)
        reader.start(); reader.join(.5)
        assert not reader.is_alive() and reads == [True]
        assert node._publication_worker.summary()['submitted'] == 0
    finally:
        release.set(); diagnostic.join(1.); reader.join(1.)


def test_shutdown_during_metadata_commit_prevents_worker_recreation_or_sdk_restart(camera):
    node, ros, _ = camera
    entered, release = threading.Event(), threading.Event(); original_params = ros.set_param
    original_camera = node.cam
    made, started = [], []
    original_make = node._make_camera
    def make(simulate):
        made.append(simulate)
        return original_make(simulate)
    node._make_camera = make
    original_camera.start = lambda: started.append(True)
    def params(name, value):
        if name == '/camera':
            entered.set(); assert release.wait(2.)
        original_params(name, value)
    ros.set_param = params
    errors = []
    def update():
        try:
            node._publish_runtime_camera_params()
        except Exception as error:
            errors.append(error)
    updater = threading.Thread(target=update, daemon=True)
    try:
        updater.start(); assert entered.wait(1.)
        assert node._publication_worker is None
        node.shutdown()
        assert original_camera.stopped and node._shutdown_requested
        release.set(); updater.join(1.)
        assert not updater.is_alive() and len(errors) == 1
        assert 'shutting down' in str(errors[0])
        assert node._publication_worker is None and not node._runtime_geometry_ready
        assert not node._start_camera_or_defer(False)
        assert not node._recover_from_read_error(RuntimeError('late read error'))
        assert made == started == []
        assert node.pub_color.messages == node.pub_depth.messages == []
    finally:
        release.set(); updater.join(1.)


@pytest.mark.parametrize('fallback', [False, True])
def test_shutdown_while_recovery_sdk_start_is_blocked_stops_the_late_started_stream(camera, fallback):
    node, ros, module = camera
    node.fallback_to_simulation = fallback
    entered, release = threading.Event(), threading.Event()
    class LateStartCamera(FakeCamera):
        def start(self):
            entered.set(); assert release.wait(2.)
            # A start that finishes after an earlier stop can make a stream
            # live again. Recovery must clean up this late completion.
            self.started, self.stopped = True, False
    module.RealSenseManager = LateStartCamera
    result, errors = [], []
    def recover():
        try:
            result.append(node._recover_from_read_error(RuntimeError('fake SDK timeout')))
        except Exception as error:
            errors.append(error)
    recovery = threading.Thread(target=recover, daemon=True)
    try:
        recovery.start(); assert entered.wait(1.)
        node.shutdown()
        release.set(); recovery.join(1.)
        assert not recovery.is_alive() and result == [False] and errors == []
        assert node.cam.stopped, 'late SDK start remained live after shutdown'
        assert node._publication_worker is None and not node._runtime_geometry_ready
    finally:
        release.set(); recovery.join(1.)
