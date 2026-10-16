"""Camera entrypoint resource contract; no real ROS node or camera is opened."""
from types import SimpleNamespace

import cv2
import pytest

from test_camera_node_recovery import load_camera_node


def fake_endpoints(module, monkeypatch, observe_threads):
    events = []

    class Node:
        def __init__(self):
            events.append(('camera_constructed', observe_threads()))

        def shutdown(self):
            events.append(('shutdown',))

        def request_shutdown(self):
            events.append(('shutdown_requested',))

        def spin(self):
            events.append(('spin', observe_threads()))

    def on_shutdown(callback):
        assert callback.__self__.__class__ is Node
        assert callback.__name__ == 'request_shutdown'
        events.append(('shutdown_registered',))

    ros = SimpleNamespace(
        init_node=lambda name: events.append(('ros_initialized', name, observe_threads())),
        loginfo=lambda *args: events.append(('log', *args)),
        on_shutdown=on_shutdown,
    )
    monkeypatch.setattr(module, 'rospy', ros)
    monkeypatch.setattr(module, 'CameraNode', Node)
    return events


def test_camera_main_bounds_real_opencv_before_ros_and_camera_start(monkeypatch):
    module = load_camera_node()
    previous = cv2.getNumThreads()
    try:
        cv2.setNumThreads(2)
        events = fake_endpoints(module, monkeypatch, cv2.getNumThreads)
        module.main()
        assert events == [
            ('ros_initialized', 'camera_node', 1),
            ('log', 'Camera OpenCV worker threads: %s', 1),
            ('camera_constructed', 1),
            ('shutdown_registered',),
            ('spin', 1),
            ('shutdown',),
        ]
    finally:
        cv2.setNumThreads(previous)


def test_camera_start_log_reports_backend_value_instead_of_requested_value(monkeypatch):
    module = load_camera_node()
    calls = []
    # An unexpected backend response must remain visible in the startup log.
    fake_cv = SimpleNamespace(setNumThreads=lambda n: calls.append(n), getNumThreads=lambda: 7)
    monkeypatch.setattr(module, 'cv2', fake_cv)
    events = fake_endpoints(module, monkeypatch, fake_cv.getNumThreads)
    module.main()
    assert calls == [1]
    assert ('log', 'Camera OpenCV worker threads: %s', 7) in events


def test_importing_camera_library_does_not_change_host_process_thread_pool(monkeypatch):
    def forbidden(_):
        pytest.fail('Importing camera_node must not configure a different host process')

    monkeypatch.setattr(cv2, 'setNumThreads', forbidden)
    module = load_camera_node()
    assert callable(module.main)
