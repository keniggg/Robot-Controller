"""Disk stalls must not block live source evidence; ROS messages round-trip losslessly."""
import io
import importlib.util
from pathlib import Path
import threading
import time
import pytest
import rospy
import rosbag
from sensor_msgs.msg import Image,JointState

path=Path(__file__).resolve().parent/'gen4_root_cause/collect_single_axis.py'
spec=importlib.util.spec_from_file_location('support_recorder_under_test',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def recorder(bag):
    r=module.Recorder.__new__(module.Recorder)
    r.lock=threading.RLock();r.bag=bag;r.fatal=None;r.active=True;r.frames={};r.counts={}
    r.states=[];r.last_joint_ns=0;r.armed=False;r.latest_task_active=False
    r.start_archive_writer()
    return r


def test_callback_keeps_source_evidence_fresh_while_disk_is_blocked(monkeypatch):
    entered=threading.Event();release=threading.Event()
    class Bag:
        def write(self,*args,**kwargs):entered.set();release.wait(2.)
    r=recorder(Bag());monkeypatch.setattr(module.rospy.Time,'now',lambda:rospy.Time(10))
    image=Image();image.header.stamp=rospy.Time(10);image.data=b'pixels'
    try:
        r.callback('/supervisor/camera/color/image_raw')(image)
        assert entered.wait(1.)
        joints=JointState();joints.header.stamp=rospy.Time(10);joints.header.frame_id='sdk_measured'
        joints.name=['Joint%d'%i for i in range(1,7)]+['right_finger'];joints.position=[0.]*7
        start=time.monotonic();r.callback('/alicia_d/accepted_joint_states')(joints)
        assert time.monotonic()-start<.1 and len(r.states)==1
        r.callback('/supervisor/camera/depth/image_raw')(image)
        assert len(r.frames[10_000_000_000])==2
        assert r.fatal is None
    finally:release.set();assert r.archive_writer.close()['archive_complete']


def test_raw_bag_roundtrip_preserves_all_pixels_headers_and_receipt_time(tmp_path):
    bag=rosbag.Bag(str(tmp_path/'roundtrip.bag'),'w',compression='lz4');r=recorder(bag)
    image=Image();image.header.stamp=rospy.Time(123,456);image.header.frame_id='camera'
    image.width=16;image.height=10;image.step=48;image.encoding='bgr8';image.data=bytes(range(240))*2
    expected=io.BytesIO();image.serialize(expected)
    r.archive_message('/image',image,rospy.Time(124,789))
    assert r.archive_writer.close()['archive_complete'];bag.close()
    with rosbag.Bag(str(tmp_path/'roundtrip.bag')) as saved:
        rows=list(saved.read_messages())
    assert len(rows)==1 and rows[0].timestamp.to_nsec()==124_000_000_789
    actual=io.BytesIO();rows[0].message.serialize(actual)
    assert expected.getvalue()==actual.getvalue()


def test_serialization_error_latches_recording_failure_and_prevents_more_admission():
    class Bag:
        def write(self,*args,**kwargs):pass
    class Invalid:
        def serialize(self,stream):raise ValueError('broken message')
    r=recorder(Bag())
    with pytest.raises(ValueError):r.archive_message('/bad',Invalid(),rospy.Time(1))
    assert 'serialization' in r.fatal
    with pytest.raises(RuntimeError):r.archive_writer.require_healthy()
    assert not r.archive_writer.close()['archive_complete']
