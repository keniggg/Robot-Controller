"""Manager opt-in integration: actual read/align/clip order, no ROS/devices.

The regional algorithm has its own real-image tests; this controlled backend
isolates whether the manager passes real ownership evidence and respects resets.
"""
from types import SimpleNamespace
import sys

import numpy as np
import pytest

from alicia_flexible_grasp.vision.realsense_manager import RealSenseManager
from test_stationary_depth import manager as global_manager, Frame, settle


class RegionalBackend:
    def __init__(self,factory):
        self.factory=factory;self.calls=[];self.reset_count=0;self.live=False
    def reset(self):self.reset_count+=1;self.live=False
    def process(self,frame,color,source,host,encoder_active,aligned_to_color=True):
        self.calls.append(dict(frame=frame,color=color,source=source,host=host,
                               encoder_active=encoder_active,aligned_to_color=aligned_to_color))
        data=np.asanyarray(frame.get_data()).copy()
        active=bool(encoder_active and aligned_to_color)
        if active:
            data[0,2]+=1  # Only one tile owns this pixel, holes remain current.
            self.live=True
        else:self.reset()
        return data,dict(enabled=True,active=active,fully_active=False,
            filtered_pixel_count=int(active),filtered_fraction=float(active)/data.size,
            owner_schema='test-exclusive-owner',regions=[dict(owner_id=0,owned_pixels=1,
                qualified=active,warming=False,filter_applied=active,history_reset=not active)])


def regional_manager(monkeypatch):
    module=SimpleNamespace(RegionalStationaryDepth=RegionalBackend)
    monkeypatch.setitem(sys.modules,'alicia_flexible_grasp.vision.regional_stationary_depth',module)
    m=RealSenseManager(mode_depth_preset_cfg=dict(stationary_temporal_enabled=True,
                                                regional_stationary_enabled=True))
    m.mode_depth_preset=SimpleNamespace(mode='unknown')
    m.stationary_depth_provider=lambda:True
    m._diagnostic_clock=lambda:10.
    color=np.zeros((2,6,3),np.uint8)
    depth=np.array([[0,299,300,1000,20000,20001],[0,299,300,1000,20000,20001]],np.uint16)
    class SDKFrame:
        def __init__(self,pixels):self.pixels=pixels
        def get_data(self):return self.pixels
        def get_frame_number(self):return 100
        def get_timestamp(self):return 1000.
        def get_frame_timestamp_domain(self):return 'hardware_clock'
        def get_profile(self):return SimpleNamespace(fps=lambda:30)
    rgb,dep=SDKFrame(color),SDKFrame(depth)
    pair=SimpleNamespace(get_color_frame=lambda:rgb,get_depth_frame=lambda:dep)
    m.pipeline=SimpleNamespace(wait_for_frames=lambda:pair)
    m.align=SimpleNamespace(process=lambda frames:frames)
    # Represents the exact align(color) instance bound by successful start().
    m._sdk_color_alignment_owner=m.align
    return m,color,depth,pair


def test_default_full_frame_path_does_not_construct_regional_backend(monkeypatch):
    def forbidden(*a,**k):raise AssertionError('default must stay full-frame')
    monkeypatch.setitem(sys.modules,'alicia_flexible_grasp.vision.regional_stationary_depth',
                        SimpleNamespace(RegionalStationaryDepth=forbidden))
    m,_=global_manager();result=settle(m,Frame(np.ones((2,2),np.uint16)))
    assert m.runtime_profile['stationary_temporal']['active']
    assert m._regional_stationary is None


@pytest.mark.parametrize('value',[None,'true',1,0,{},[]])
def test_opt_in_requires_literal_boolean(value):
    with pytest.raises(ValueError,match='regional_stationary_enabled'):
        RealSenseManager(mode_depth_preset_cfg={'regional_stationary_enabled':value})


@pytest.mark.parametrize('missing',['no_align','not_bound','changed_owner','flag_only','config_false'])
def test_equal_dimensions_or_flag_without_current_sdk_align_cannot_enable_regions(monkeypatch,missing):
    m,color,depth,pair=regional_manager(monkeypatch)
    if missing=='no_align':m.align=None
    elif missing=='not_bound':m._sdk_color_alignment_owner=None
    elif missing=='changed_owner':m.align=SimpleNamespace(process=lambda frames:frames)
    elif missing=='config_false':m.align_depth_to_color=False
    if missing=='flag_only':
        m._sdk_color_alignment_owner=None
        result=m._filter_stationary_depth(Frame(depth),
            dict(stages_sec={},failed_stage=None,sdk_color_alignment_applied=True),color=color,
            source={'frame_number':100})
        np.testing.assert_array_equal(result.get_data(),depth)
    else:
        _,result=m.read()
        np.testing.assert_array_equal(result,[[0,0,300,1000,20000,0]]*2)
    status=m.runtime_profile['stationary_temporal']
    assert not status['alignment_proven'] and not status['regional_output_active']
    assert not status['active'] and not status['whole_frame_stationarity_qualified']
    assert not m._regional_stationary.calls[-1]['aligned_to_color']


def test_aligned_regional_result_is_clipped_then_rectified_without_changing_original(monkeypatch):
    m,color,depth,_=regional_manager(monkeypatch)
    seen=[]
    def rectify(rgb,clipped):
        seen.append((rgb.copy(),clipped.copy()))
        return rgb.copy(),clipped.copy()
    m.pair_rectifier=SimpleNamespace(apply=rectify)
    before=depth.copy();returned_rgb,returned_depth=m.read()
    np.testing.assert_array_equal(depth,before)
    np.testing.assert_array_equal(returned_rgb,color)
    np.testing.assert_array_equal(returned_depth,[[0,0,301,1000,20000,0],[0,0,300,1000,20000,0]])
    np.testing.assert_array_equal(seen[0][1],returned_depth)
    call=m._regional_stationary.calls[-1]
    assert call['aligned_to_color'] and call['encoder_active']
    assert call['source']['frame_number']==100
    status=m.runtime_profile['stationary_temporal']
    assert status['regional_output_active'] and status['filtered_pixel_count']==1
    assert status['filtered_fraction']==1/12 and not status['fully_active']
    assert not status['active'] and not status['whole_frame_stationarity_qualified']
    assert status['regions'][0]['qualified']
    assert m.last_read_diagnostics['sdk_color_alignment_applied']
    assert m.last_read_diagnostics['regional_stationary_evidence']['filtered_pixel_count']==1


@pytest.mark.parametrize('fault',['encoder','carton','disabled','missing_pair','exception','mode_switch','stop'])
def test_global_invalidations_clear_regional_state_and_activity(monkeypatch,fault):
    m,_,_,_=regional_manager(monkeypatch);m.read()
    regions=m._regional_stationary;assert regions.live
    if fault=='encoder':m.stationary_depth_provider=lambda:False;m.read()
    elif fault=='carton':m.mode_depth_preset.mode='carton';m.read()
    elif fault=='disabled':m.mode_depth_preset_cfg['stationary_temporal_enabled']=False;m.read()
    elif fault=='missing_pair':
        m.pipeline.wait_for_frames=lambda:SimpleNamespace(get_color_frame=lambda:None,get_depth_frame=lambda:None)
        assert m.read()==(None,None)
    elif fault=='exception':
        def fail():raise RuntimeError('camera read failed')
        m.pipeline.wait_for_frames=fail
        with pytest.raises(RuntimeError):m.read()
    elif fault=='mode_switch':
        m.mode_depth_preset.apply=lambda mode:True
        m.mode_depth_preset.evidence=lambda:dict(mode='carton')
        m.set_depth_preset_mode('carton')
    else:m._stop_pipeline();assert m._sdk_color_alignment_owner is None
    assert not regions.live and regions.reset_count>0
    status=m.runtime_profile['stationary_temporal']
    assert not status['regional_output_active'] and not status['active']
    assert status['filtered_pixel_count']==0 and status['filtered_fraction']==0


def test_switching_back_to_default_discards_regions_and_starts_global_warmup(monkeypatch):
    m,_,_,_=regional_manager(monkeypatch);m.read();regions=m._regional_stationary
    m.mode_depth_preset_cfg['regional_stationary_enabled']=False
    # The tiny untextured fixture cannot qualify the global visual gate.
    m.read()
    assert regions.reset_count and not regions.live
    assert m._regional_stationary is None and m._stationary_temporal is None
    assert not m.runtime_profile['stationary_temporal']['active']


def test_filter_factory_preserves_exact_original_policy(monkeypatch):
    m,_,_,_=regional_manager(monkeypatch)
    from test_stationary_depth import Filter
    made=[]
    def factory():
        f=Filter();made.append(f);return f
    m.rs=SimpleNamespace(temporal_filter=factory,option=SimpleNamespace(
        filter_smooth_alpha='alpha',filter_smooth_delta='delta',holes_fill='persistence'))
    m._new_regional_temporal_filter(alpha=.4,delta=100,persistence=0)
    assert made[0].options==dict(alpha=.4,delta=100,persistence=0)
    with pytest.raises(ValueError,match='policy changed'):
        m._new_regional_temporal_filter(alpha=.2,delta=100,persistence=0)
    assert len(made)==1


def test_real_regional_algorithm_through_read_prewarm_align_clip_and_rectify():
    from test_regional_stationary_contract import Frame as ProfileFrame
    from test_visual_stationary import texture
    from test_stationary_depth_warmup import AveragingFilter
    m=RealSenseManager(mode_depth_preset_cfg=dict(stationary_temporal_enabled=True,
                                                regional_stationary_enabled=True))
    m.mode_depth_preset=SimpleNamespace(mode='unknown')
    m.stationary_depth_provider=lambda:True
    m.rs=SimpleNamespace(temporal_filter=AveragingFilter,option=SimpleNamespace(
        filter_smooth_alpha='alpha',filter_smooth_delta='delta',holes_fill='persistence'))
    index=[-1];image=np.repeat(texture()[:,:,None],3,axis=2);originals=[]
    m._diagnostic_clock=lambda:10.+index[0]/30.
    class SDKFrame(ProfileFrame):
        def get_profile(self):
            h,w=self.pixels.shape[:2]
            intrinsic=SimpleNamespace(width=w,height=h,fx=400.,fy=400.,ppx=w/2.,ppy=h/2.,
                coeffs=[0.]*5,model='none')
            return SimpleNamespace(stream_type=lambda:'depth',stream_index=lambda:0,
                format=lambda:'z16',unique_id=lambda:1,fps=lambda:30,
                as_video_stream_profile=lambda:SimpleNamespace(get_intrinsics=lambda:intrinsic))
        def get_frame_number(self):return 100+index[0]
        def get_timestamp(self):return 1000.+index[0]*1000./30.
        def get_frame_timestamp_domain(self):return 'hardware_clock'
    def wait():
        index[0]+=1
        data=np.full(image.shape[:2],2000+50*(index[0]%2),np.uint16);data[0,0]=0
        rgb,depth=SDKFrame(image),SDKFrame(data)
        originals.append((depth,data.copy()))
        return SimpleNamespace(get_color_frame=lambda:rgb,get_depth_frame=lambda:depth)
    m.pipeline=SimpleNamespace(wait_for_frames=wait)
    m.align=SimpleNamespace(process=lambda frames:frames)
    m._sdk_color_alignment_owner=m.align
    rectified=[]
    def rectify(color,depth):rectified.append(depth.copy());return color,depth
    m.pair_rectifier=SimpleNamespace(apply=rectify)
    for i in range(12):
        color,depth=m.read()
        np.testing.assert_array_equal(originals[-1][0].get_data(),originals[-1][1])
        np.testing.assert_array_equal(rectified[-1],depth)
        np.testing.assert_array_equal(color,image)
        assert depth[0,0]==0
        status=m.runtime_profile['stationary_temporal']
        assert not status['active'] and not status['whole_frame_stationarity_qualified']
        if i<11:
            np.testing.assert_array_equal(depth,originals[-1][1])
            assert not status['regional_output_active']
        else:
            assert status['regional_output_active'] and status['fully_active']
            assert status['filtered_fraction']==1.
            assert np.all((depth[1:]>2000)&(depth[1:]<2050))


def test_disabling_both_regional_and_temporal_clears_old_activity_diagnostic(monkeypatch):
    m,_,_,_=regional_manager(monkeypatch);m.read()
    assert m.runtime_profile['stationary_temporal']['regional_output_active']
    previous=m._regional_stationary
    m.mode_depth_preset_cfg.update(regional_stationary_enabled=False,stationary_temporal_enabled=False)
    m.read()
    assert previous.reset_count and not previous.live and m._regional_stationary is None
    status=m.runtime_profile['stationary_temporal']
    assert not status['regional_output_active'] and not status['active'] and not status['fully_active']
    assert status['filtered_pixel_count']==0 and status['filtered_fraction']==0.
    assert not status['history_warming_up'] and status['regions']==[]
