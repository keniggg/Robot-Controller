"""Signal-hook ordering with fake ROS/SDK; never initialize real hardware."""
import threading
from types import SimpleNamespace

import pytest

from test_camera_async_publication import camera
from test_camera_node_recovery import load_camera_node


@pytest.mark.parametrize('outcome', ['return', 'exception', 'keyboard_interrupt'])
def test_main_hook_does_not_stop_inside_active_sdk_call_and_finally_closes(monkeypatch, outcome):
    module = load_camera_node()
    events, hooks = [], []
    active = {'sdk_call': False}

    class FakeNode:
        def __init__(self):
            self.requested = False

        def request_shutdown(self):
            self.requested = True
            events.append('request')

        def spin(self):
            events.append('sdk_enter')
            active['sdk_call'] = True
            try:
                hooks[0]()  # Model synchronous rospy signal hook in read().
                assert self.requested and events == ['sdk_enter', 'request']
                if outcome == 'exception':
                    raise RuntimeError('SDK operation failed')
                if outcome == 'keyboard_interrupt':
                    raise KeyboardInterrupt('interrupted')
            finally:
                active['sdk_call'] = False
                events.append('sdk_exit')

        def shutdown(self):
            assert active['sdk_call'] is False
            events.append('stop')

    monkeypatch.setattr(module, 'CameraNode', FakeNode)
    monkeypatch.setattr(module, 'cv2', SimpleNamespace(setNumThreads=lambda n: None,
                                                     getNumThreads=lambda: 1))
    monkeypatch.setattr(module, 'rospy', SimpleNamespace(init_node=lambda *a: None,
        loginfo=lambda *a: None, on_shutdown=lambda callback: hooks.append(callback)))
    if outcome == 'return':
        module.main()
    else:
        with pytest.raises(RuntimeError if outcome == 'exception' else KeyboardInterrupt):
            module.main()
    assert events == ['sdk_enter', 'request', 'sdk_exit', 'stop']
    assert hooks[0].__name__ == 'request_shutdown'


def test_main_registration_failure_still_closes_constructed_node(monkeypatch):
    module = load_camera_node()
    events = []
    class FakeNode:
        def request_shutdown(self):
            pass
        def shutdown(self):
            events.append('stop')
        def spin(self):
            raise AssertionError('spin must not start')
    def fail(callback):
        raise RuntimeError('registration failure')
    monkeypatch.setattr(module, 'CameraNode', FakeNode)
    monkeypatch.setattr(module, 'cv2', SimpleNamespace(setNumThreads=lambda n: None,
                                                     getNumThreads=lambda: 1))
    monkeypatch.setattr(module, 'rospy', SimpleNamespace(init_node=lambda *a: None,
        loginfo=lambda *a: None, on_shutdown=fail))
    with pytest.raises(RuntimeError, match='registration failure'):
        module.main()
    assert events == ['stop']


def test_request_is_nonblocking_while_lifecycle_lock_is_held(camera):
    node, _, _ = camera
    camera_owner = node.cam
    worker = node._publication_worker
    timer = node._runtime_status_timer
    done = threading.Event()
    def request():
        node.request_shutdown()
        done.set()
    with node._publication_lifecycle_lock:
        thread = threading.Thread(target=request, daemon=True)
        thread.start()
        assert done.wait(.5), 'request hook tried to acquire lifecycle lock'
    thread.join(1.)
    assert node._shutdown_requested and not camera_owner.stopped
    assert node._publication_worker is worker and not timer.stopped
    node.shutdown()
    assert camera_owner.stopped and timer.stopped


def test_real_node_request_during_read_leaves_sdk_owned_until_read_returns(camera):
    node, _, _ = camera
    events = []
    owner = node.cam
    old_read, old_stop = owner.read, owner.stop
    def read():
        events.append('read_enter')
        node.request_shutdown()
        assert not owner.stopped
        pair = old_read()
        events.append('read_exit')
        return pair
    def stop():
        assert events[-1] == 'read_exit'
        events.append('stop')
        return old_stop()
    owner.read, owner.stop = read, stop
    try:
        try:
            node.spin()
        finally:
            node.shutdown()
        assert events == ['read_enter', 'read_exit', 'stop']
    finally:
        owner.stop = old_stop  # Fixture teardown also uses explicit shutdown.


def test_explicit_shutdown_remains_immediate_and_repeatable(camera):
    node, _, _ = camera
    owner, worker, timer = node.cam, node._publication_worker, node._runtime_status_timer
    node.shutdown()
    assert owner.stopped and timer.stopped and node._shutdown_requested
    assert not node._camera_started and not node._runtime_geometry_ready
    first_summary = worker.summary()
    node.shutdown()
    assert owner.stopped and timer.stopped and node._publication_worker is worker
    assert worker.summary()['published'] == first_summary['published']
    assert not node._recover_from_read_error(RuntimeError('late failure'))


def test_request_during_recovery_start_stops_after_start_stack_returns(camera):
    node, _, _ = camera
    factory = node._make_camera
    events = []
    def make(simulate):
        replacement = factory(simulate)
        old_start, old_stop = replacement.start, replacement.stop
        def start():
            events.append('start_enter')
            node.request_shutdown()
            assert not replacement.stopped
            old_start()
            events.append('start_exit')
        def stop():
            assert 'start_exit' in events
            events.append('stop')
            old_stop()
        replacement.start, replacement.stop = start, stop
        return replacement
    node._make_camera = make
    with pytest.raises(RuntimeError, match='shutting down'):
        node._start_camera(False)
    assert events == ['start_enter', 'start_exit', 'stop']
    assert node.cam.stopped and not node._runtime_geometry_ready
