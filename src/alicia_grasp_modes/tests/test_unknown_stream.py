"""Ingress remains live during expensive image processing; stamps stay exact."""
import importlib.util
from pathlib import Path
import threading
import types

import pytest


path = Path(__file__).resolve().parents[1] / 'scripts' / 'unknown_tabletop_perception.py'
spec = importlib.util.spec_from_file_location('unknown_stream_test', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def node():
    value = module.UnknownTabletopPerception.__new__(module.UnknownTabletopPerception)
    value.lock, value.input_lock = threading.RLock(), threading.RLock()
    value.mode, value.generation, value.selection_ns = 'unknown', 7, 100
    value.latest, value.last_consumed_ns, value.last_receive_wall = None, 110, 0.
    return value


def image(stamp):
    return types.SimpleNamespace(header=types.SimpleNamespace(
        stamp=types.SimpleNamespace(to_nsec=lambda:stamp)))


def test_latest_pair_is_received_while_processing_lock_is_busy():
    n = node()
    first, second = image(120), image(130)
    done = threading.Event()
    def receive():
        n.frame(first, first)
        n.frame(second, second)
        done.set()
    with n.lock:
        thread = threading.Thread(target=receive)
        thread.start()
        received_without_processing_unlock = done.wait(1.)
    thread.join(timeout=1.)
    assert received_without_processing_unlock
    assert n.latest == (second, second)
    assert n.last_receive_wall > 0


@pytest.mark.parametrize('color,depth', [(100,100),(110,110),(120,121)])
def test_input_lock_does_not_weaken_stamp_or_generation_boundaries(color, depth):
    n = node()
    n.frame(image(color), image(depth))
    assert n.latest is None and n.last_receive_wall == 0.


def test_inactive_mode_does_not_accept_buffered_images():
    n = node()
    n.mode = 'carton'
    n.frame(image(120), image(120))
    assert n.latest is None


def test_late_delivery_cannot_replace_a_newer_unprocessed_pair():
    n = node()
    newest = image(130)
    n.frame(newest, newest)
    n.frame(image(120), image(120))
    assert n.latest == (newest, newest)


def test_failed_tracked_refinement_withholds_raw_mask_and_recovers_next_frame(monkeypatch):
    import numpy as np
    from cv_bridge import CvBridge
    from unittest.mock import Mock
    n = node()
    n.max_age, n.tf_wait, n.minimum_quality = .8, .15, .5
    n.base_frame, n.camera_frame = 'base_link', 'camera_link'
    n.config, n.rgb_instance_refinement = module.Config(), True
    n.bridge, n.last_shape = CvBridge(), (20, 20)
    n.anchor_source_stamp_ns = 120
    raw = types.SimpleNamespace(mask=np.full((20,20),255,np.uint8), quality=.9,
        center_uv=[10.,10.], bbox=(0,0,20,20), depth_m=.3,
        position_camera=[.3,0.,0.], position_base=[.3,0.,0.])
    n.tracker = types.SimpleNamespace(anchor=raw, last=raw, plane_base=np.array([0.,0.,1.,0.]),
        choose=lambda *_:(raw,'tracked'), unavailable=Mock())
    n.buffer = types.SimpleNamespace(lookup_transform=lambda *_:object())
    n.status = Mock()
    n.pubs = {key:Mock() for key in ('object','object_mask','object_detected','raw_object_detected',
                                    'object_pose_camera','object_pose_base')}
    monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda:module.rospy.Time(10)))
    monkeypatch.setattr(module.rospy, 'get_param', lambda *_:dict(align_depth_to_color=True,fx=400,fy=400,cx=10,cy=10))
    monkeypatch.setattr(module, 'transform_matrix', lambda _:np.eye(4))
    monkeypatch.setattr(module, 'segment_with_support_fallback', lambda *_,**__:object())
    def rejected(*_):
        raise ValueError('rgb_instance_reaches_search_boundary')
    monkeypatch.setattr(module, 'refine_candidate', rejected)
    def pair(ns):
        rgb = n.bridge.cv2_to_imgmsg(np.zeros((20,20,3),np.uint8), encoding='bgr8')
        depth = n.bridge.cv2_to_imgmsg(np.full((20,20),.3,np.float32), encoding='32FC1')
        for msg in (rgb,depth):
            msg.header.stamp = module.rospy.Time(9,ns)
            msg.header.frame_id = 'camera_link'
        return rgb,depth
    n.latest = pair(900000000)
    n.process(None)
    assert not n.pubs['object'].publish.call_args[0][0].detected
    published = n.pubs['object_mask'].publish.call_args[0][0]
    assert not np.any(n.bridge.imgmsg_to_cv2(published, desired_encoding='mono8'))
    n.pubs['object_pose_base'].publish.assert_not_called()
    n.tracker.unavailable.assert_not_called()
    assert n.tracker.anchor is raw
    assert 'rgb_depth_refinement_rejected' in n.status.call_args[0][1]
    monkeypatch.setattr(module, 'refine_candidate', lambda *_:raw)
    n.latest = pair(950000000)
    n.process(None)
    assert n.pubs['object'].publish.call_args[0][0].detected
    assert n.pubs['object'].publish.call_args[0][0].header.stamp == module.rospy.Time(9,950000000)
