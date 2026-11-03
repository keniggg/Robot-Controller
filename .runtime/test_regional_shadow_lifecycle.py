"""Independent shadow lifetime tests; no ROS subscriptions or SDK devices."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
import pytest

from test_camera_stage_observer import Clock, Frame, Pipeline, Align, Spatial

path=Path(__file__).with_name('regional_stationary_shadow.py')
spec=importlib.util.spec_from_file_location('regional_shadow_lifecycle',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


@pytest.fixture
def setup(tmp_path,monkeypatch):
    clock=Clock();created=[]
    class Evidence:
        def __init__(self,_):
            self.closed=0;self.context_checks=0;self.fail_guard=False
            self.guard=SimpleNamespace(check=self.check,snapshot=lambda:{'actual':'fixture'})
            created.append(self)
        def check(self,*args,**kwargs):
            if self.fail_guard:raise ValueError('measured evidence changed')
            return True
        def check_context(self):self.context_checks+=1
        def now_ns(self):return 10000000000
        def at_anchor(self,stamp):return dict(anchor=stamp,evidence_complete=True)
        def close(self):self.closed+=1
    class Copier:
        def __init__(self,_):self.calls=0
        def copy(self,frame):self.calls+=1;return Frame(frame.get_data().copy())
    class Regional:
        def __init__(self,factory):self.factory=factory;self.calls=[];self.resets=0;self.fail=False
        def reset(self):self.resets+=1
        def process(self,frame,color,source,host,encoder_active,aligned_to_color=True):
            self.calls.append((frame,encoder_active,aligned_to_color))
            if self.fail:raise ValueError('regional test failure')
            return frame.get_data()+np.uint16(100),dict(active=True,fully_active=True,regions=[])
    monkeypatch.setattr(module.ab,'RosEvidence',Evidence)
    monkeypatch.setattr(module.ab,'PrivateDepthCopy',Copier)
    monkeypatch.setitem(sys.modules,'rospy',SimpleNamespace())
    monkeypatch.setitem(sys.modules,'alicia_flexible_grasp.vision.regional_stationary_depth',
                        SimpleNamespace(RegionalStationaryDepth=Regional))
    original_init=module.stage.CameraStageObserver.__init__
    def offline_init(self,*args,**kwargs):
        kwargs.update(start_thread=False,clock=clock,warmup=0.,stable_reads=1,samples=2,interval=.5)
        original_init(self,*args,**kwargs)
    monkeypatch.setattr(module.stage.CameraStageObserver,'__init__',offline_init)
    class Camera:
        def __init__(self):
            self.pipeline=Pipeline();self.align=Align();self._sdk_color_alignment_owner=self.align
            self.align_depth_to_color=True;self.depth_filters=[Spatial()]
            self.applied_depth_filters=[{'name':'spatial'}]
            self.runtime_profile={'stationary_temporal':{'active':True}};self.depth_scale=.0001
            self.mode_depth_preset=SimpleNamespace(mode='unknown')
            self.mode_depth_preset_cfg={'regional_stationary_enabled':False}
            self.rs=SimpleNamespace(frame_metadata_value=SimpleNamespace(actual_exposure='exposure'))
            self._diagnostic_clock=clock;self.pair_rectifier=None
            self.last_read_diagnostics=None;self.calls=0;self.failure=None;self.complete=True
            self.in_filter=None;self.original_frames=[]
        def _clip_depth_range(self,depth):return depth.copy()
        def _filter_stationary_depth(self,frame,diagnostic=None,color=None,source=None):
            if self.in_filter:self.in_filter()
            self.original_frames.append((frame,frame.get_data().copy()))
            diagnostic['stationary_rgb_evidence']={'qualified':True,'reason':'stationary'}
            return Frame(frame.get_data()+np.uint16(30))
        def read(self):
            self.calls+=1
            frames=self.pipeline.wait_for_frames();frames=self.align.process(frames)
            if self.failure:raise self.failure
            depth=frames.get_depth_frame()
            for filt in self.depth_filters:depth=filt.process(depth)
            diagnostic={}
            depth=self._filter_stationary_depth(depth,diagnostic,color=frames.color.data,
                source=dict(frame_number=17,device_timestamp_ms=1234.,timestamp_domain='hardware_clock',stream_fps=30))
            diagnostic['result']='complete' if self.complete else 'rejected_pair'
            self.last_read_diagnostics=diagnostic
            self.last_result=(frames.color.data.copy(),depth.get_data().copy()) if self.complete else (None,None)
            return self.last_result
    original=Camera.read
    camera=Camera();observer=module.RegionalShadow(Camera,tmp_path/'out')
    yield SimpleNamespace(camera=camera,observer=observer,clock=clock,cls=Camera,
                          original=original,evidence=created,Copier=Copier,Regional=Regional)
    observer.shutdown();Camera.read=original


@pytest.mark.parametrize('stage',['evidence','copier','regional'])
def test_initialization_failure_restores_read_and_executes_production_once(setup,monkeypatch,stage):
    c=setup
    def fail(*args,**kwargs):raise RuntimeError('initialization '+stage)
    if stage=='evidence':monkeypatch.setattr(module.ab,'RosEvidence',fail)
    elif stage=='copier':monkeypatch.setattr(module.ab,'PrivateDepthCopy',fail)
    else:
        monkeypatch.setitem(sys.modules,'alicia_flexible_grasp.vision.regional_stationary_depth',
                            SimpleNamespace(RegionalStationaryDepth=fail))
    result=c.camera.read()
    assert result is c.camera.last_result and c.camera.calls==1
    assert c.observer.closed and c.observer.close_reason=='regional_shadow_initialization_failed'
    assert c.observer.inflight==0 and c.cls.read is c.original
    assert '_filter_stationary_depth' not in c.camera.__dict__
    assert np.all(result[1]==2460)
    if stage!='evidence':assert c.evidence[0].closed>=1
    assert c.camera.read() is c.camera.last_result and c.camera.calls==2


def test_selected_double_tap_preserves_production_and_records_different_shadow(setup):
    c=setup;cam=c.camera;obs=c.observer
    first=cam.read();c.clock.step(.5)
    pipeline,align,filters=cam.pipeline,cam.align,cam.depth_filters
    returned=cam.read()
    assert returned is cam.last_result and np.all(returned[1]==2460)
    row=obs.queue.get_nowait()
    assert row['complete'] and row['production_output_replaced'] is False
    assert np.all(row['arrays']['pre_stationary_depth']==2430)
    assert np.all(row['arrays']['post_stationary_depth']==2460)
    assert np.all(row['arrays']['regional_shadow_depth']==2530)
    assert np.all(row['arrays']['regional_shadow_final_depth']==2530)
    assert np.all(row['arrays']['final_depth']==2460)
    assert len(obs.regional.calls)==2 and all(call[1:] == (True,True) for call in obs.regional.calls)
    for frame,before in cam.original_frames:np.testing.assert_array_equal(frame.get_data(),before)
    assert cam.pipeline is pipeline and cam.align is align and cam.depth_filters is filters
    assert '_filter_stationary_depth' not in cam.__dict__ and obs.current_row is None and obs.inflight==0
    assert cam.calls==2 and cam.pipeline.calls==2


@pytest.mark.parametrize('selected',[False,True])
def test_deadline_inside_original_filter_keeps_class_patched_until_outer_tap_removed(setup,selected):
    c=setup;cam=c.camera;obs=c.observer
    if selected:cam.read();c.clock.step(.5)
    observations=[]
    def close_during_original():
        observations.append((obs.inflight,obs.current_row is not None))
        assert obs.inflight==2 and '_filter_stationary_depth' in cam.__dict__
        obs.finish('injected_deadline')
        assert c.cls.read is obs.wrapper
    cam.in_filter=close_during_original
    returned=cam.read()
    assert returned is cam.last_result and np.all(returned[1]==2460)
    assert observations==[(2,selected)]
    assert obs.inflight==0 and obs.current_row is None
    assert c.cls.read is c.original and '_filter_stationary_depth' not in cam.__dict__


@pytest.mark.parametrize('fault',['shadow','guard','production','incomplete'])
def test_failure_cleanup_restores_all_proxies_and_preserves_original_exception(setup,fault):
    c=setup;cam=c.camera;obs=c.observer
    cam.read();c.clock.step(.5)
    pipeline,align,filters=cam.pipeline,cam.align,cam.depth_filters
    if fault=='shadow':obs.regional.fail=True
    elif fault=='guard':obs.evidence.fail_guard=True
    elif fault=='production':cam.failure=RuntimeError('same production error object')
    else:cam.complete=False
    if fault=='production':
        with pytest.raises(RuntimeError) as error:cam.read()
        assert error.value is cam.failure
    else:
        returned=cam.read()
        assert returned is cam.last_result
        if fault=='incomplete':assert returned==(None,None)
        else:assert np.all(returned[1]==2460)
    assert obs.inflight==0 and obs.current_row is None and obs.regional.resets>=1
    assert cam.pipeline is pipeline and cam.align is align and cam.depth_filters is filters
    assert '_filter_stationary_depth' not in cam.__dict__
    if fault in ('shadow','guard'):
        assert obs.closed and c.cls.read is c.original and not obs.counts['original_read_errors']
    else:
        obs.finish('test_finished');assert c.cls.read is c.original
    assert obs.counts['original_read_errors']==int(fault=='production')
