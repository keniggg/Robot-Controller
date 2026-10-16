"""Independent black-box history isolation checks, using real visual gates.

No ROS, SDK device, motion interface, or mocked positive stationarity decision.
The fake SDK only provides immutable frames and an identifiable temporal filter.
"""
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from alicia_flexible_grasp.vision.realsense_manager import RealSenseManager
from test_stationary_depth_warmup import AveragingFilter
from test_visual_stationary import texture, shifted


@pytest.fixture(autouse=True, scope='module')
def bounded_opencv_threads():
    previous = cv2.getNumThreads()
    cv2.setNumThreads(1)
    yield
    cv2.setNumThreads(previous)


class SDKFrame:
    def __init__(self, stream, index, pixels, *, offset_ms=0., fps=30., domain='hardware_clock',
                 uid=None, focal=400., missing_number=False):
        self.stream, self.index, self.pixels = stream, index, pixels
        self.offset_ms, self.fps, self.domain = offset_ms, fps, domain
        self.uid = uid if uid is not None else (1 if stream == 'color' else 2)
        self.focal, self.missing_number = focal, missing_number

    def get_data(self):
        return self.pixels

    def get_frame_number(self):
        if self.missing_number:
            raise RuntimeError('source frame counter unavailable')
        return (100 if self.stream == 'color' else 1000)+self.index

    def get_timestamp(self):
        return 1000.+self.index*(1000./30.)+self.offset_ms+(.2 if self.stream == 'depth' else 0.)

    def get_frame_timestamp_domain(self):
        return self.domain

    def get_profile(self):
        h,w=self.pixels.shape[:2]
        intrinsic=SimpleNamespace(width=w,height=h,fx=self.focal,fy=self.focal,
            ppx=w/2.,ppy=h/2.,coeffs=[0.]*5,model='none')
        return SimpleNamespace(stream_type=lambda:self.stream,stream_index=lambda:0,
            format=lambda:'bgr8' if self.stream=='color' else 'z16',unique_id=lambda:self.uid,
            fps=lambda:self.fps,as_video_stream_profile=lambda:SimpleNamespace(get_intrinsics=lambda:intrinsic))


class Rig:
    def __init__(self):
        self.manager=RealSenseManager(mode_depth_preset_cfg=dict(
            stationary_temporal_enabled=True,regional_stationary_enabled=True))
        self.now=10.;self.failure=None;self.stage_delay={};self.encoder=True;self.pair=None
        self.color=np.repeat(texture()[:,:,None],3,axis=2)
        self.base_calls=0;self.align_calls=0;self.made=[]
        m=self.manager
        m._diagnostic_clock=lambda:self.now
        m._diagnostic_wall_clock=lambda:10000.+self.now
        m._diagnostic_thread_clock=lambda:0.
        def apply_mode(mode):
            changed=m.mode_depth_preset.mode!=mode;m.mode_depth_preset.mode=mode;return changed
        m.mode_depth_preset=SimpleNamespace(mode='unknown',apply=apply_mode,evidence=lambda:{'mode':m.mode_depth_preset.mode})
        m.stationary_depth_provider=lambda:self.stage('stationary_evidence',self.encoder)
        def create():
            f=AveragingFilter();self.made.append(f);return f
        m.rs=SimpleNamespace(temporal_filter=create,option=SimpleNamespace(
            filter_smooth_alpha='alpha',filter_smooth_delta='delta',holes_fill='persistence'))
        m.pipeline=SimpleNamespace(wait_for_frames=lambda:self.stage('wait',self.pair),stop=lambda:None)
        m.align=SimpleNamespace(process=self.align)
        m._sdk_color_alignment_owner=m.align
        m.depth_filters=[SimpleNamespace(process=self.base_filter)]
        m.applied_depth_filters=[dict(name='spatial',options={})]
        m.pair_rectifier=SimpleNamespace(apply=lambda c,d:self.stage('rectify',(c,d)))

    def stage(self, name, value):
        self.now+=self.stage_delay.get(name,0.)
        if self.failure==name:
            raise RuntimeError('injected '+name)
        return value

    def align(self, frames):
        self.align_calls+=1
        return self.stage('align',frames)

    def base_filter(self, frame):
        self.base_calls+=1
        return self.stage('base_filter',frame)

    def feed(self, color_index, depth_index=None, *, host=None, color_pixels=None,
             color_kwargs=None, depth_kwargs=None, missing=None):
        depth_index=color_index if depth_index is None else depth_index
        self.now=10.+max(color_index,depth_index)/30. if host is None else host
        color=SDKFrame('color',color_index,self.color if color_pixels is None else color_pixels,
                       **(color_kwargs or {}))
        self.raw_depth=np.full(self.color.shape[:2],2000+50*(depth_index%2),np.uint16)
        self.raw_depth[0,0]=0
        depth=SDKFrame('depth',depth_index,self.raw_depth,**(depth_kwargs or {}))
        self.pair=SimpleNamespace(get_color_frame=lambda:None if missing=='color' else color,
            get_depth_frame=lambda:None if missing=='depth' else depth)
        return self.manager.read()

    def warm(self):
        for i in range(12):
            self.feed(i)
        assert self.manager.runtime_profile['stationary_temporal']['fully_active']
        self.old_filters=tuple(r['filter'] for r in self.manager._regional_stationary._regions)
        self.old_counts=tuple(f.calls for f in self.old_filters)
        self.last_complete=self.manager._previous_completed_raw_frames
        return self

    def quarantine(self, reverse=False):
        result=self.feed(12,11,host=10.+11.5/30.) if reverse else self.feed(11,12,host=10.+11.5/30.)
        assert result==(None,None)
        assert self.manager._previous_completed_raw_frames is self.last_complete

    def assert_old_history_unused(self, before=None):
        assert tuple(f.calls for f in self.old_filters)==(self.old_counts if before is None else before)

    def assert_current_raw_output(self, result):
        assert result[0] is not None
        np.testing.assert_array_equal(result[1],self.raw_depth)
        assert not self.manager.runtime_profile['stationary_temporal']['regional_output_active']
        assert result[1][0,0]==0


@pytest.mark.parametrize('reverse',[False,True])
def test_only_next_complete_pair_continues_real_history_without_feeding_rejected_pixels(reverse):
    r=Rig().warm();base,aligned=r.base_calls,r.align_calls
    r.quarantine(reverse)
    assert (r.base_calls,r.align_calls)==(base,aligned)
    r.assert_old_history_unused()
    result=r.feed(12)
    assert all(region['filter'] is f for region,f in zip(r.manager._regional_stationary._regions,r.old_filters))
    assert tuple(f.calls for f in r.old_filters)==tuple(n+1 for n in r.old_counts)
    assert r.manager.runtime_profile['stationary_temporal']['fully_active']
    assert np.all(result[1][1:]>r.raw_depth[1:])
    assert result[1][0,0]==0
    assert (r.base_calls,r.align_calls)==(base+1,aligned+1)


def test_successor_is_bound_to_last_complete_not_advanced_rejected_source():
    r=Rig().warm();r.quarantine()
    # Relative to the rejected pair depth+1, but relative to last complete
    # depth+2: it must not reuse history even though color is exactly +1.
    result=r.feed(12,13,depth_kwargs={'offset_ms':-1000./30.})
    r.assert_current_raw_output(result);r.assert_old_history_unused()


@pytest.mark.parametrize('depth_index',[10,11,13,14])
def test_depth_regression_repeat_or_skip_cannot_hide_behind_continuous_color(depth_index):
    r=Rig().warm();r.quarantine()
    result=r.feed(12,depth_index,host=10.4,
        depth_kwargs={'offset_ms':(12-depth_index)*1000./30.})
    r.assert_current_raw_output(result);r.assert_old_history_unused()


def test_repeated_interleavings_do_not_renew_the_original_host_window():
    r=Rig().warm()
    for host in (10.383,10.41,10.45,10.47):
        assert r.feed(11,12,host=host)==(None,None)
        assert r.manager._previous_completed_raw_frames is r.last_complete
    result=r.feed(12,host=10.49)
    r.assert_current_raw_output(result);r.assert_old_history_unused()


def test_delayed_preprocessing_is_checked_at_actual_visual_observation_time():
    r=Rig().warm();r.quarantine();r.stage_delay['align']=.12
    result=r.feed(12,host=10.4)
    r.assert_current_raw_output(result);r.assert_old_history_unused()


@pytest.mark.parametrize('fault',['reset','mode_roundtrip','stop','pipeline','align','base_filter','rectifier','depth_scale','image_geometry'])
def test_history_owner_or_epoch_change_cannot_reactivate_old_filters(fault):
    r=Rig().warm();r.quarantine();m=r.manager
    if fault=='reset':m._reset_stationary_history()
    elif fault=='mode_roundtrip':m.set_depth_preset_mode('carton');m.set_depth_preset_mode('unknown')
    elif fault=='stop':m._stop_pipeline();m._sdk_color_alignment_owner=m.align
    elif fault=='pipeline':m.pipeline=SimpleNamespace(wait_for_frames=lambda:r.pair,stop=lambda:None)
    elif fault=='align':m.align=SimpleNamespace(process=r.align);m._sdk_color_alignment_owner=m.align
    elif fault=='base_filter':m.depth_filters=[SimpleNamespace(process=r.base_filter)]
    elif fault=='rectifier':m.pair_rectifier=SimpleNamespace(apply=lambda c,d:(c,d))
    elif fault=='depth_scale':m.depth_scale*=2.
    else:r.color=r.color[:-2,:,:].copy()
    result=r.feed(12)
    r.assert_current_raw_output(result);r.assert_old_history_unused()


@pytest.mark.parametrize('stream',['color','depth'])
@pytest.mark.parametrize('change',[{'uid':77},{'focal':410.},{'fps':29.},{'domain':'global_time'}])
def test_either_stream_profile_or_clock_change_invalidates_retained_history(stream,change):
    r=Rig().warm();r.quarantine()
    kwargs={stream+'_kwargs':change}
    if 'domain' in change:
        kwargs={'color_kwargs':change,'depth_kwargs':change}
    result=r.feed(12,**kwargs)
    if result[0] is not None:r.assert_current_raw_output(result)
    r.assert_old_history_unused()


@pytest.mark.parametrize('fault',['missing_color','missing_depth','missing_counter','wrong_domain','skew_not_adjacent','same_counter_changed_stamp'])
def test_missing_or_unproven_rejected_pair_cannot_be_quarantined(fault):
    r=Rig().warm()
    kwargs={}
    if fault.startswith('missing_') and fault!='missing_counter':kwargs['missing']=fault[8:]
    elif fault=='missing_counter':kwargs['color_kwargs']={'missing_number':True}
    elif fault=='wrong_domain':kwargs['color_kwargs']={'domain':'system_time'}
    elif fault=='same_counter_changed_stamp':kwargs['color_kwargs']={'offset_ms':5.}
    assert r.feed(11,13 if fault=='skew_not_adjacent' else 12,host=10.383,**kwargs)==(None,None)
    result=r.feed(12)
    r.assert_current_raw_output(result);r.assert_old_history_unused()


@pytest.mark.parametrize('stage',['wait','align','base_filter','stationary_evidence','rectify'])
def test_any_processing_exception_after_pair_acceptance_clears_isolated_history(stage):
    r=Rig().warm();r.quarantine();r.failure=stage
    with pytest.raises(RuntimeError,match='injected '+stage):r.feed(12)
    assert r.manager.last_read_diagnostics['result']=='error'
    assert all(region['filter'] is None for region in r.manager._regional_stationary._regions)
    counts=tuple(f.calls for f in r.old_filters)
    r.failure=None
    result=r.feed(13)
    r.assert_current_raw_output(result);r.assert_old_history_unused(counts)


@pytest.mark.parametrize('clock_jump_ms',[-100.,100.])
def test_immediate_counters_cannot_hide_device_clock_jump(clock_jump_ms):
    r=Rig().warm();r.quarantine()
    result=r.feed(12,color_kwargs={'offset_ms':clock_jump_ms},
                  depth_kwargs={'offset_ms':clock_jump_ms})
    r.assert_current_raw_output(result);r.assert_old_history_unused()


@pytest.mark.parametrize('fault',['encoder','rgb_motion','rgb_texture'])
def test_original_stationarity_permission_must_still_pass_on_the_complete_pair(fault):
    r=Rig().warm();r.quarantine();kwargs={}
    if fault=='encoder':r.encoder=False
    elif fault=='rgb_motion':kwargs['color_pixels']=shifted(r.color,y=2.)
    else:kwargs['color_pixels']=np.zeros_like(r.color)
    result=r.feed(12,**kwargs)
    r.assert_current_raw_output(result);r.assert_old_history_unused()
