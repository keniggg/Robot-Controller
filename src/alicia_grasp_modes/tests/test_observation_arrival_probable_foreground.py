"""Arrival label proposals keep measured identity, geometry and retry guards."""
import cv2
import numpy as np
import pytest

from alicia_grasp_modes.rgb_instance import instance_mask_from_seed
from alicia_grasp_modes.tabletop import Config, _project
from test_observation_arrival_rgb_retry import scene, grabcut_stub


def geometry(seed, depth=None):
    if depth is None:
        depth=np.full(seed.shape,.3,np.float32)
        depth[seed>0]=.28
    points=_project(depth,[400.,400.,320.,240.])
    return points,depth,.3-points[...,0],np.array([-1.,0.,0.]),Config()


def test_arrival_geometry_uses_original_small_core_in_two_bounded_searches(monkeypatch):
    color,seed=scene();calls=grabcut_stub(monkeypatch,[('left',),'valid'])
    mask,evidence=instance_mask_from_seed(color,seed,geometry=geometry(seed),observation_arrival_retry=True)
    np.testing.assert_array_equal(mask,seed)
    assert [shape for shape,_ in calls]==[(116,116),(116,132)]
    assert evidence['rgb_search_initial_margin_px']==48
    assert evidence['rgb_trusted_core_search_margin_px']==16
    assert evidence['rgb_search_margins_ltrb_px']==[64,48,48,48]
    assert evidence['rgb_search_attempts']==2
    core=np.zeros(seed.shape,bool);core[231:249,311:329]=True
    for _,observed in calls:np.testing.assert_array_equal(observed,core)


@pytest.mark.parametrize('outcome,reason,count',[
    ('boundary','rgb_instance_reaches_search_boundary',2),
    ('fragmented','rgb_seed_component_ambiguous_or_fragmented',1),
    ('large','rgb_instance_not_supported_by_depth_seed',1)])
def test_arrival_failures_do_not_get_unbounded_or_non_boundary_retries(monkeypatch,outcome,reason,count):
    color,seed=scene();calls=grabcut_stub(monkeypatch,[outcome]*count)
    with pytest.raises(ValueError,match='^'+reason+'$'):
        instance_mask_from_seed(color,seed,geometry=geometry(seed),observation_arrival_retry=True)
    assert len(calls)==count


def test_arrival_larger_context_still_refuses_unobserved_rgb_before_grabcut(monkeypatch):
    color,seed=scene();calls=grabcut_stub(monkeypatch,[])
    valid=np.ones(seed.shape,bool);valid[190,300]=False
    with pytest.raises(ValueError,match='^rgb_search_contains_unobserved_rgb$'):
        instance_mask_from_seed(color,seed,geometry=geometry(seed),rgb_valid=valid,observation_arrival_retry=True)
    assert calls==[]


def captured_labels(monkeypatch,color,seed,geom,enabled):
    captured=[]
    def inspect_labels(patch,labels,*args):
        captured.append(labels.copy())
        raise RuntimeError('labels captured')
    monkeypatch.setattr(cv2,'grabCut',inspect_labels)
    with pytest.raises(RuntimeError,match='labels captured'):
        instance_mask_from_seed(color,seed,geometry=geom,observation_arrival_retry=enabled)
    assert len(captured)==1
    return captured[0]


@pytest.mark.parametrize('fault',['valid','support','missing','nonfinite_depth','too_near',
    'too_far','nonfinite_xyz','nonfinite_height','too_low','too_high'])
def test_only_current_valid_measured_height_adds_probable_not_hard_foreground(monkeypatch,fault):
    color,seed=scene();points,depth,heights,normal,config=geometry(seed)
    # Outside the selected seed's 7px probable dilation, inside its original ROI.
    y,x=240,340
    depth[y,x]=.28;points[y,x]=[-.28,0.,0.];heights[y,x]=.02
    if fault=='support':heights[y,x]=0.
    elif fault=='missing':depth[y,x]=0.
    elif fault=='nonfinite_depth':depth[y,x]=np.nan
    elif fault=='too_near':depth[y,x]=config.depth_min_m/2
    elif fault=='too_far':depth[y,x]=config.depth_max_m+1
    elif fault=='nonfinite_xyz':points[y,x]=np.nan
    elif fault=='nonfinite_height':heights[y,x]=np.nan
    elif fault=='too_low':heights[y,x]=(config.foreground_min_m+config.support_threshold_m)/2
    elif fault=='too_high':heights[y,x]=config.foreground_max_m+1
    with np.errstate(invalid='ignore'):
        observed=captured_labels(monkeypatch,color,seed,(points,depth,heights,normal,config),True)
    assert observed[y-(230-48),x-(310-48)]==(
        cv2.GC_PR_FGD if fault=='valid' else cv2.GC_BGD if fault=='support' else cv2.GC_PR_BGD)
    # The added measured proposal cannot become a hard foreground seed.
    assert np.count_nonzero(observed==cv2.GC_FGD)==18*18
    assert not np.any(observed[:3]!=cv2.GC_BGD)
    assert not np.any(observed[-3:]!=cv2.GC_BGD)


def test_default_geometry_and_no_geometry_paths_keep_original_first_labels(monkeypatch):
    color,seed=scene();geom=geometry(seed)
    geom[1][240,340]=.28;geom[0][240,340]=[-.28,0.,0.];geom[2][240,340]=.02
    observed=captured_labels(monkeypatch,color,seed,geom,False)
    assert observed.shape==(52,52)
    assert observed[26,46]==cv2.GC_PR_BGD
    plain=captured_labels(monkeypatch,color,seed,None,False)
    arrival_without_geometry=captured_labels(monkeypatch,color,seed,None,True)
    np.testing.assert_array_equal(plain,arrival_without_geometry)


def adjacent_scene(connected):
    color=np.full((480,640,3),(20,20,180),np.uint8)
    seed=np.zeros(color.shape[:2],np.uint8);seed[230:250,300:322]=255
    color[seed>0]=230
    depth=np.full(seed.shape,.3,np.float32);depth[seed>0]=.28
    if connected:
        # A larger second measured object, joined only by a narrow RGB bridge.
        color[220:265,332:377]=230;depth[220:265,332:377]=.28
        color[239:242,322:332]=230
    else:
        color[230:250,337:357]=230;depth[230:250,337:357]=.28
    return color,seed,depth


@pytest.mark.parametrize('neighbour_colour',[(230,230,230),(20,180,20)])
def test_separate_same_height_object_does_not_join_trusted_component(neighbour_colour):
    color,seed,depth=adjacent_scene(False)
    color[230:250,337:357]=neighbour_colour
    before=[a.copy() for a in (color,seed,depth)]
    mask,evidence=instance_mask_from_seed(color,seed,geometry=geometry(seed,depth),observation_arrival_retry=True)
    assert np.count_nonzero(mask[230:250,300:322])>=400
    assert not np.any(mask[:,337:357])
    assert evidence['trusted_core_retained_fraction']>=.85
    for actual,expected in zip((color,seed,depth),before):np.testing.assert_array_equal(actual,expected)


def test_rgb_bridge_over_measured_support_cannot_join_neighbour():
    color,seed,depth=adjacent_scene(True)
    mask,_=instance_mask_from_seed(color,seed,geometry=geometry(seed,depth),observation_arrival_retry=True)
    assert not np.any(mask[:,332:377])
    assert np.count_nonzero(mask[230:250,300:322])>=400


def test_rgb_bridge_with_missing_depth_is_rejected_by_original_area_guard():
    color,seed,depth=adjacent_scene(True)
    depth[239:242,322:332]=0.  # No measured support label can trivially cut this bridge.
    with pytest.raises(ValueError,match='^rgb_instance_not_supported_by_depth_seed$'):
        instance_mask_from_seed(color,seed,geometry=geometry(seed,depth),observation_arrival_retry=True)
