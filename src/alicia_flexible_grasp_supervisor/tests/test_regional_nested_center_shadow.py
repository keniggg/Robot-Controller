"""Independent diagnostic crop must never supply depth or motion authority."""
import copy
import json

import cv2
import numpy as np
import pytest

from alicia_flexible_grasp.vision.regional_stationary_depth import RegionalStationaryDepth
from test_regional_stationary_depth import Factory, Frame, run, source, texture, warm
from test_stationary_pair_history_independent import Rig


@pytest.fixture(autouse=True)
def one_opencv_thread():
    old=cv2.getNumThreads();cv2.setNumThreads(1)
    yield
    cv2.setNumThreads(old)


class DiagnosticRegionalStationaryDepth(RegionalStationaryDepth):
    """Tests explicitly request the isolated diagnostic; production defaults off."""
    def process(self, frame, color, source, host, encoder_active, aligned_to_color=False):
        returned=super().process(frame,color,source,host,encoder_active,aligned_to_color)
        if returned[1]['reason']=='regional_observation':
            returned[1]['nested_center_shadow']=self._observe_nested_center_shadow(color,source,host)
        return returned


class NoShadow(DiagnosticRegionalStationaryDepth):
    def _observe_nested_center_shadow(self, *args):
        return dict(diagnostic_only=True, skipped_by_test_baseline=True)


def original_report(report):
    result=copy.deepcopy(report);result.pop('nested_center_shadow')
    return result


def test_normal_production_never_calls_or_constructs_nested_observer(monkeypatch):
    engine=RegionalStationaryDepth(Factory())
    def forbidden(*args):pytest.fail('permanent diagnostic computation')
    monkeypatch.setattr(engine,'_observe_nested_center_shadow',forbidden)
    for i in range(14):
        _,report=run(engine,texture(),i)
        assert report['nested_center_shadow']['reason']=='diagnostic_disabled'
        assert report['nested_center_shadow']['qualified'] is False
        assert engine._nested_shadow_visual is None and engine._nested_shadow_bounds is None


@pytest.mark.parametrize('size,bounds',[(640,[224,144,416,336]),(320,[112,72,208,168]),
                                     (1280,[448,288,832,672])])
def test_same_real_visual_policy_on_centered_half_width_half_height(size,bounds):
    factory=Factory();engine=DiagnosticRegionalStationaryDepth(factory)
    _,report=warm(engine,texture(size,size*3//4))
    shadow=report['nested_center_shadow']
    assert shadow['bounds_xyxy']==bounds
    assert shadow['qualified'] and shadow['visual']['qualified']
    assert shadow['visual']['window_sec']==.35 and shadow['visual']['max_motion_px']==.25
    assert shadow['visual']['covered_quadrants']>=3
    assert shadow['diagnostic_only'] and not shadow['pixels_changed']
    assert not shadow['used_for_depth_permission'] and not shadow['motion_authorized']
    assert not shadow['temporal_filter_created'] and len(factory.filters)==5
    assert report['owner_schema']=='fixed_center_02_08_01_09_then_four_quadrants_v1'
    json.dumps(report,allow_nan=False)


def test_static_nested_crop_never_authorizes_moving_original_center_pixels():
    factory=Factory();engine=DiagnosticRegionalStationaryDepth(factory);rgb=texture();warm(engine,rgb)
    moved=rgb.copy();moved[48:432,128:220]=np.roll(rgb[48:432,128:220],6,axis=0)
    output,report=run(engine,moved,12)
    assert report['regions'][0]['visual']['reason']=='visual_motion'
    assert report['nested_center_shadow']['qualified'] is True
    assert report['regions'][0]['filter_applied'] is False
    assert np.all(output[48:432,128:512]==1000)
    assert len(factory.filters)==5


@pytest.mark.parametrize('case',['static','periphery','whole_motion','holes','encoder',
                                'profile','geometry','source_gap','host_gap'])
def test_original_five_region_reports_pixels_and_factory_calls_are_bitwise_unchanged(case):
    factories=[Factory(),Factory()];engines=[DiagnosticRegionalStationaryDepth(factories[0]),NoShadow(factories[1])]
    rgb=texture()
    for i in range(14):
        color=rgb;kwargs={};frame_kwargs={}
        if i>=12:
            if case=='periphery':
                color=rgb.copy();color[:,:128]=np.roll(color[:,:128],6,axis=0)
            elif case=='whole_motion':color=np.roll(rgb,2,axis=0)
            elif case=='encoder':kwargs['encoder_active']=False
            elif case=='profile':frame_kwargs['fx']=401.
            elif case=='geometry':color=texture(320,240)
            elif case=='source_gap':kwargs['source']=source(i+1)
            elif case=='host_gap':kwargs['host']=11.+i/30.
        pixels=np.full(color.shape[:2],1000,np.uint16)
        if case=='holes':pixels[::7,::9]=0
        pairs=[run(engine,color,i,frame=Frame(pixels.copy(),**frame_kwargs),**kwargs) for engine in engines]
        assert np.array_equal(pairs[0][0],pairs[1][0])
        assert original_report(pairs[0][1])==original_report(pairs[1][1])
        assert [f.calls for f in factories[0].filters]==[f.calls for f in factories[1].filters]


@pytest.mark.parametrize('kind',['reset','encoder','alignment','color','source','profile','geometry',
                                'host_gap','sdk_skip'])
def test_original_global_reset_paths_also_revoke_shadow_history(kind):
    engine=DiagnosticRegionalStationaryDepth(Factory());rgb=texture();warm(engine,rgb)
    old=engine._nested_shadow_visual
    assert old._count==12
    if kind=='reset':
        engine.reset();assert engine._nested_shadow_visual is None
        output,report=run(engine,rgb,12)
    else:
        kwargs={}
        if kind=='encoder':kwargs['encoder_active']=False
        elif kind=='alignment':kwargs['aligned_to_color']=False
        elif kind=='color':rgb=rgb.astype(np.float32)
        elif kind=='source':kwargs['source']=dict(source(12),device_timestamp_ms=float('nan'))
        elif kind=='profile':kwargs['frame']=Frame(np.full(rgb.shape[:2],1000,np.uint16),profile_id=8)
        elif kind=='geometry':rgb=texture(320,240)
        elif kind=='host_gap':kwargs['host']=11.
        elif kind=='sdk_skip':kwargs['source']=source(14)
        output,report=run(engine,rgb,12,**kwargs)
    assert engine._nested_shadow_visual is not old
    assert report['nested_center_shadow']['qualified'] is False
    assert not report['nested_center_shadow']['pixels_changed']
    if engine._nested_shadow_visual is not None:assert engine._nested_shadow_visual._count<=1


@pytest.mark.parametrize('fault',['raise','mutate_then_raise'])
def test_shadow_failure_is_unknown_and_cannot_change_any_real_pixels_or_report(fault):
    factory=Factory();engine=DiagnosticRegionalStationaryDepth(factory);baseline=NoShadow(Factory())
    rgb=texture();warm(engine,rgb);warm(baseline,rgb);original=rgb.copy();info=source(12)
    class Broken:
        def observe(self,color,source,host):
            if fault=='mutate_then_raise':color[:]=0;source.clear()
            raise RuntimeError('injected shadow-only exception')
    engine._nested_shadow_visual=Broken()
    output,report=run(engine,rgb,12,source=info)
    expected,oldreport=run(baseline,rgb,12,source=source(12))
    assert np.array_equal(output,expected) and np.array_equal(rgb,original)
    assert original_report(report)==original_report(oldreport)
    assert info==source(12) and len(factory.filters)==5
    assert report['nested_center_shadow']['reason']=='shadow_observation_error'
    assert report['nested_center_shadow']['qualified'] is False
    assert report['nested_center_shadow']['visual'] is None
    assert engine._nested_shadow_visual is None
    _,later=run(engine,rgb,13)
    assert later['nested_center_shadow']['visual']['reason']=='warming_up'


def test_original_filter_failure_also_clears_shadow_without_masking_original_error():
    factory=Factory();engine=DiagnosticRegionalStationaryDepth(factory);rgb=texture();warm(engine,rgb)
    factory.fault='dtype'
    with pytest.raises(ValueError,match='output_geometry_changed'):run(engine,rgb,12)
    assert engine._nested_shadow_visual is None


def test_invalid_small_shadow_texture_does_not_affect_original_region_decisions():
    rgb=np.zeros((120,160,3),np.uint8)
    engine=DiagnosticRegionalStationaryDepth(Factory());baseline=NoShadow(Factory())
    for i in range(12):
        output,report=run(engine,rgb,i);other,oldreport=run(baseline,rgb,i)
        assert np.array_equal(output,other) and original_report(report)==original_report(oldreport)
        assert report['nested_center_shadow']['qualified'] is False


def test_shadow_state_is_not_part_of_manager_pair_history_owner_and_global_reset_clears_it():
    rig=Rig();rig.warm()
    manager=rig.manager;engine=manager._regional_stationary
    token=manager._stationary_history_owner()
    assert token is not None and engine._nested_shadow_visual is None
    # A diagnostic-only observer must remain excluded even if manually installed.
    from alicia_flexible_grasp.vision.visual_stationary import VisualStationaryWindow
    engine._nested_shadow_visual=VisualStationaryWindow()
    engine._nested_shadow_bounds=engine._nested_center_bounds()
    assert manager._stationary_history_owner()==token
    engine._nested_shadow_visual=None;engine._nested_shadow_bounds=None
    assert manager._stationary_history_owner()==token
    manager._reset_stationary_history()
    assert engine._nested_shadow_visual is None
