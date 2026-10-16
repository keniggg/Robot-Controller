"""Delayed measured TF must keep exact source time and original image lifetime."""
from types import SimpleNamespace as NS
from unittest.mock import Mock
import numpy as np
import pytest
from cv_bridge import CvBridge
from test_unknown_stream import module, node


@pytest.mark.parametrize('age,limit,wait,wanted', [
    (.02,.8,.4,.4), (.65,.8,.4,.15), (.8,.8,.4,0.), (.02,.8,.15,.15)])
def test_wait_is_bounded_by_unchanged_source_lifetime(age,limit,wait,wanted):
    assert module.exact_tf_wait_budget(age,limit,wait)==pytest.approx(wanted)


@pytest.mark.parametrize('age,limit,wait', [
    (-.001,.8,.4), (.801,.8,.4), (float('nan'),.8,.4),
    (.1,float('inf'),.4), (.1,0.,.4), (.1,.8,0.), (.1,.8,float('nan'))])
def test_invalid_or_stale_input_cannot_request_a_transform(age,limit,wait):
    with pytest.raises(ValueError):module.exact_tf_wait_budget(age,limit,wait)


def prepared(monkeypatch,age=.02,delay=.28):
    n=node();clock=[10.+age];calls=[]
    n.max_age,n.tf_wait,n.minimum_quality=.8,.4,.5
    n.base_frame,n.camera_frame='base_link','camera_link'
    n.config,n.rgb_instance_refinement=module.Config(),True
    n.bridge,n.last_shape=CvBridge(),(20,20)
    n.anchor_source_stamp_ns=9_000_000_000
    raw=NS(mask=np.full((20,20),255,np.uint8),quality=.9,center_uv=[10.,10.],
        bbox=(0,0,20,20),depth_m=.3,position_camera=np.array([.3,0.,0.]),
        position_base=np.array([.3,0.,0.]),extent_m=np.array([.02,.01,.01]),
        support_footprint_base=np.zeros((4,3)))
    n.tracker=NS(anchor=raw,last=raw,lost=False,plane_base=np.array([0.,0.,1.,0.]),
        choose=Mock(return_value=(raw,'tracked')))
    n.tracker.unavailable=Mock(side_effect=lambda:setattr(n.tracker,'lost',True))
    def lookup(base,camera,stamp,timeout):
        calls.append((base,camera,stamp.to_nsec(),timeout.to_sec()))
        clock[0]+=delay
        return object()
    n.buffer=NS(lookup_transform=lookup)
    n.status=Mock()
    n.pubs={key:Mock() for key in ('object','object_mask','object_detected',
        'raw_object_detected','object_pose_camera','object_pose_base')}
    monkeypatch.setattr(module.rospy.Time,'now',staticmethod(lambda:module.rospy.Time.from_sec(clock[0])))
    monkeypatch.setattr(module.rospy,'get_param',lambda *_:dict(align_depth_to_color=True,fx=400,fy=400,cx=10,cy=10))
    monkeypatch.setattr(module.rospy,'logwarn',Mock())
    monkeypatch.setattr(module.rospy,'logwarn_throttle',Mock())
    monkeypatch.setattr(module,'transform_matrix',lambda _:np.eye(4))
    # These tests stub geometry to isolate exact TF and source lifetime. The
    # stub must carry the existing qualified-current-support contract; a bare
    # object fails in tracking before the intended timing checkpoint is reached.
    segmentation=Mock(return_value=NS(metrics={'support_current_audit_available': True}))
    monkeypatch.setattr(module,'segment_with_support_fallback',segmentation)
    monkeypatch.setattr(module,'refine_candidate',lambda *_:raw)
    rgb=n.bridge.cv2_to_imgmsg(np.zeros((20,20,3),np.uint8),encoding='bgr8')
    depth=n.bridge.cv2_to_imgmsg(np.full((20,20),.3,np.float32),encoding='32FC1')
    for msg in (rgb,depth):msg.header.stamp=module.rospy.Time(10);msg.header.frame_id='camera_link'
    n.latest=(rgb,depth)
    return n,clock,calls,segmentation,raw


def test_measured_feedback_arriving_after_old_150ms_wait_preserves_exact_source(monkeypatch):
    n,_,calls,_,_=prepared(monkeypatch)
    n.process(None)
    assert calls==[('base_link','camera_link',10_000_000_000,.4)]
    obj=n.pubs['object'].publish.call_args[0][0]
    assert obj.detected and obj.header.stamp.to_nsec()==10_000_000_000
    assert n.pubs['object_mask'].publish.call_args[0][0].header.stamp==obj.header.stamp
    assert n.first_loss is None


def test_lookup_cannot_extend_original_800ms_age_budget(monkeypatch):
    n,_,calls,_,_=prepared(monkeypatch,age=.65,delay=.1)
    n.process(None)
    assert calls[0][3]==pytest.approx(.15,abs=2e-9)
    assert n.pubs['object'].publish.call_args[0][0].detected


def test_late_lookup_return_is_rejected_before_geometry(monkeypatch):
    n,_,_,segmentation,_=prepared(monkeypatch,delay=.79)
    n.process(None)
    segmentation.assert_not_called()
    assert not n.pubs['object'].publish.call_args[0][0].detected
    assert n.first_loss['reason']=='perception_failure: source_frame_stale_or_future'
    assert n.tracker.lost


def test_expensive_rgb_processing_cannot_publish_an_expired_observation(monkeypatch):
    n,clock,_,_,raw=prepared(monkeypatch)
    def refine(*_):clock[0]+=.6;return raw
    monkeypatch.setattr(module,'refine_candidate',refine)
    n.process(None)
    assert not n.pubs['object'].publish.call_args[0][0].detected
    n.pubs['object_pose_base'].publish.assert_not_called()
    assert n.tracker.lost


def test_missing_exact_tf_keeps_original_permanent_loss_behavior(monkeypatch):
    n,_,_,segmentation,_=prepared(monkeypatch)
    def missing(*_):raise module.tf2_ros.ExtrapolationException('future sample still missing')
    n.buffer.lookup_transform=missing
    n.process(None)
    segmentation.assert_not_called()
    assert n.tracker.lost and n.first_loss['reason'].startswith('missing_exact_tf:')
    assert not n.pubs['object'].publish.call_args[0][0].detected
