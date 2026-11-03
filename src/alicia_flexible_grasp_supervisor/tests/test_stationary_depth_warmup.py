"""Private temporal prewarm does not grant early filtering permission."""
from pathlib import Path
import os
from types import SimpleNamespace

import numpy as np
import pytest

from test_stationary_depth import Frame, manager, feed, settle
from test_visual_stationary import shifted, source


class AveragingFilter:
    def __init__(self):
        self.options={};self.previous=None;self.calls=0
    def set_option(self,key,value):self.options[key]=value
    def process(self,frame):
        current=np.asanyarray(frame.get_data()).copy()
        self.calls+=1
        if self.previous is not None:
            valid=(current!=0)&(self.previous!=0)
            current[valid]=np.round(.4*current[valid]+.6*self.previous[valid]).astype(np.uint16)
        self.previous=current.copy()
        return Frame(current)


def averaging_manager():
    m,made=manager()
    def create():
        result=AveragingFilter();made.append(result);return result
    m.rs.temporal_filter=create
    return m,made


def test_warmup_returns_original_frame_but_first_qualified_frame_uses_history():
    m,made=averaging_manager()
    for index in range(12):
        pixels=np.array([[0,2000+50*(index%2)]],np.uint16)
        frame=Frame(pixels.copy())
        result=feed(m,frame)
        np.testing.assert_array_equal(frame.get_data(),pixels)
        assert not result.get_data()[0,0]
        status=m.runtime_profile['stationary_temporal']
        if index<11:
            assert result is frame and status['history_warming_up'] and not status['active']
        else:
            assert result is not frame and status['active'] and not status['history_warming_up']
            assert 2000<result.get_data()[0,1]<2050
    assert len(made)==1 and made[0].calls==12
    assert made[0].options=={'alpha':.4,'delta':100.,'persistence':0.}


@pytest.mark.parametrize('stage',['warming','active'])
@pytest.mark.parametrize('fault',['motion','missing_rgb','texture','source_gap','timestamp_gap',
    'domain','host_pause','encoder','mode','explicit_reset','silent_visual_reset','insufficient_tracks'])
def test_every_broken_qualification_cuts_history_and_requires_new_window(monkeypatch,stage,fault):
    m,made=averaging_manager();frame=Frame(np.array([[0,2000]],np.uint16))
    for _ in range(4 if stage=='warming' else 12):feed(m,frame)
    original=m._stationary_temporal;old_anchor=m._stationary_visual_anchor
    assert original is not None
    if fault=='motion':result=feed(m,frame,shifted(m.test_color,y=1.))
    elif fault=='missing_rgb':result=m._filter_stationary_depth(frame)
    elif fault=='texture':result=feed(m,frame,np.zeros_like(m.test_color))
    elif fault=='encoder':
        m.stationary_depth_provider=lambda:False;result=feed(m,frame)
        m.stationary_depth_provider=lambda:True
    elif fault=='mode':
        m.mode_depth_preset.mode='carton';result=feed(m,frame);m.mode_depth_preset.mode='unknown'
    elif fault=='explicit_reset':m._reset_stationary_history();result=frame
    elif fault=='silent_visual_reset':
        m._visual_stationary.reset();result=feed(m,frame)
    elif fault=='insufficient_tracks':
        import cv2
        original_flow=cv2.calcOpticalFlowPyrLK
        def missing(*args,**kwargs):
            points,status,error=original_flow(*args,**kwargs)
            status[:]=0
            return points,status,error
        with monkeypatch.context() as patch:
            patch.setattr(cv2,'calcOpticalFlowPyrLK',missing)
            result=feed(m,frame)
    else:
        index=m.test_index;m.test_index+=1
        info=source(index);host=10.+index/30.
        if fault=='source_gap':info['frame_number']+=1
        elif fault=='timestamp_gap':info['device_timestamp_ms']+=50.
        elif fault=='domain':info['timestamp_domain']='global_time'
        elif fault=='host_pause':host+=.2
        m._diagnostic_clock=lambda:host
        result=m._filter_stationary_depth(frame,color=m.test_color,source=info)
    assert result is frame and not m.runtime_profile['stationary_temporal']['active']
    if fault=='silent_visual_reset':
        assert m._stationary_temporal is not original
        assert m._stationary_visual_anchor is not old_anchor
        assert m._stationary_temporal.calls==1
    else:
        assert m._stationary_temporal is None and m._stationary_visual_anchor is None
    # A fresh window is required regardless of whether the old one was active.
    for _ in range(3):
        result=feed(m,frame)
        assert result is frame and not m.runtime_profile['stationary_temporal']['active']
        assert m._stationary_temporal is not original
    settle(m,frame)
    assert m.runtime_profile['stationary_temporal']['active']
    assert m._stationary_temporal is not original


def test_private_warmup_hole_violation_is_rejected_before_any_output():
    m,made=manager();frame=Frame(np.array([[0,2000]],np.uint16))
    assert feed(m,frame) is frame
    made[0].fill_holes=True
    with pytest.raises(ValueError,match='filled current depth holes'):
        feed(m,frame)
    assert m._stationary_temporal is None and m._stationary_visual_anchor is None
    assert m._visual_stationary._anchor is None


@pytest.mark.parametrize('fault',['missing_pair','bad_skew','read_error'])
def test_real_read_failure_clears_prequalified_history(monkeypatch,fault):
    m,made=averaging_manager();frame=Frame(np.array([[0,2000]],np.uint16))
    for _ in range(4):assert feed(m,frame) is frame
    assert m._stationary_temporal is not None
    if fault=='read_error':
        def fail():raise RuntimeError('missing acquisition')
        m.pipeline=SimpleNamespace(wait_for_frames=fail)
        with pytest.raises(RuntimeError):m.read()
    else:
        class SDKFrame:
            def __init__(self,stamp):self.stamp=stamp
            def get_frame_number(self):return 105
            def get_timestamp(self):return self.stamp
            def get_frame_timestamp_domain(self):return 'hardware_clock'
            def get_profile(self):return SimpleNamespace(fps=lambda:30)
        rgb=SDKFrame(1200.)
        depth=SDKFrame(1234.) if fault=='bad_skew' else None
        frames=SimpleNamespace(get_color_frame=lambda:rgb,get_depth_frame=lambda:depth)
        m.pipeline=SimpleNamespace(wait_for_frames=lambda:frames)
        assert m.read()==(None,None)
    assert m._stationary_temporal is None and m._stationary_visual_anchor is None
    assert m._visual_stationary._anchor is None
    assert not m.runtime_profile['stationary_temporal']['active']
    assert not m.runtime_profile['stationary_temporal']['history_warming_up']


@pytest.mark.skipif(os.environ.get('ALICIA_OFFLINE_SDK_COPY_TEST') != '1',
                    reason='Opt-in SDK software-frame test initializes an isolated udev monitor')
def test_actual_sdk_software_frames_prewarm_without_changing_source_or_holes(monkeypatch):
    # In-memory librealsense software device only; no physical enumeration,
    # camera pipeline, ROS, serial port, or USB device is created.
    sdk=Path(__file__).resolve().parents[3]/'.runtime'/'realsense-2.56.5'
    monkeypatch.syspath_prepend(str(sdk))
    rs=pytest.importorskip('pyrealsense2')
    assert rs.__version__=='2.56.5'
    def forbidden(*args,**kwargs):raise AssertionError('physical SDK entry forbidden')
    monkeypatch.setattr(rs,'context',forbidden);monkeypatch.setattr(rs,'pipeline',forbidden)
    m,_=manager();m.rs=rs
    device=rs.software_device();sensor=device.add_sensor('stationary_prewarm_memory_test')
    stream=rs.video_stream();stream.type=rs.stream.depth;stream.index=0;stream.uid=1
    stream.width=16;stream.height=12;stream.fps=30;stream.bpp=2;stream.fmt=rs.format.z16
    k=rs.intrinsics();k.width=16;k.height=12;k.fx=k.fy=10;k.ppx=8;k.ppy=6
    k.model=rs.distortion.none;k.coeffs=[0.]*5;stream.intrinsics=k
    profile=sensor.add_video_stream(stream).as_video_stream_profile()
    sensor.add_read_only_option(rs.option.depth_units,.0001)
    queue=rs.frame_queue(1);sensor.open(profile);sensor.start(queue)
    def sdk_frame(index,base):
        data=np.full((12,16),base,np.uint16);data[0,0]=0
        incoming=rs.software_video_frame();incoming.pixels=data;incoming.bpp=2;incoming.stride=32
        incoming.timestamp=1000.+index*1000./30.;incoming.domain=rs.timestamp_domain.hardware_clock
        incoming.frame_number=100+index;incoming.profile=profile;incoming.depth_units=.0001
        sensor.on_video_frame(incoming)
        return queue.wait_for_frame(1000),data
    try:
        for index in range(12):
            frame,pixels=sdk_frame(index,2000+50*(index%2))
            result=feed(m,frame)
            np.testing.assert_array_equal(np.asanyarray(frame.get_data()),pixels)
            assert np.asanyarray(result.get_data())[0,0]==0
            if index<11:assert result is frame
            else:
                assert result is not frame
                assert np.all((np.asanyarray(result.get_data())[1:]>2000)&
                              (np.asanyarray(result.get_data())[1:]<2050))
        # A motion frame revokes the old history; a later stationary window
        # must contain only new values, including when they are within delta.
        old=m._stationary_temporal
        for index in range(12,25):
            frame,pixels=sdk_frame(index,2080)
            result=feed(m,frame,shifted(m.test_color,y=1.))
            np.testing.assert_array_equal(np.asanyarray(frame.get_data()),pixels)
            np.testing.assert_array_equal(np.asanyarray(result.get_data()),pixels)
            if index==12:assert m._stationary_temporal is None and result is frame
        assert m._stationary_temporal is not old and m.runtime_profile['stationary_temporal']['active']
    finally:
        sensor.stop();sensor.close()
