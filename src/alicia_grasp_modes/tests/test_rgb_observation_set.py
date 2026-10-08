from dataclasses import replace

import numpy as np
import pytest

from alicia_grasp_modes.rgb_observation_region import (
    CenterSearchPolicy,RgbObservationRegion,RgbObservationRegions)
from alicia_grasp_modes.rgb_observation_set import (
    select_rgb_interest_set,RgbObservationSetTracker,observation_set_digest,
    observation_set_metadata,observation_set_metadata_digest)
from alicia_grasp_modes.rgb_observation_source import digest


POLICY=CenterSearchPolicy((.5,.5))


def region(x,y=40,score=0.):
    mask=np.zeros((120,160),bool);mask[y:y+12,x:x+12]=True
    return RgbObservationRegion(mask,(x,y,12,12),(x+5.5,y+5.5),144,
        .9,.8,30.,.05,score,('gray',),None)


def proposals(*regions,reason=None):
    ordered=sorted(regions,key=lambda item:item.score)
    ambiguous=len(ordered)>1 and ordered[1].score-ordered[0].score<=.12
    return RgbObservationRegions(tuple(regions),None if ambiguous or not regions else
        regions.index(ordered[0]),reason or ('ambiguous_rgb_regions' if ambiguous else
        'observation_region_found' if regions else 'no_compact_rgb_region'),(120,160),len(regions))


def observe(tracker,items,frame,**kwargs):
    values=dict(policy=POLICY,source_id='same-source',generation=1,
                stamp_ns=10**9+frame*100_000_000,joints_rad=[0.]*6)
    values.update(kwargs)
    return tracker.observe(items,**values)


def test_original_margin_keeps_all_competitors_as_original_objects():
    a,b,c=region(30),region(60,score=.11),region(90,score=.3)
    result=select_rgb_interest_set(proposals(c,b,a),POLICY)
    assert len(result)==2 and result[0] is a and result[1] is b
    result=select_rgb_interest_set(proposals(c,a),POLICY)
    assert len(result)==1 and result[0] is a


def test_exact_old_margin_boundary_is_still_a_set():
    assert len(select_rgb_interest_set(proposals(region(30),region(60,score=.12)),POLICY))==2


@pytest.mark.parametrize('data',[
    proposals(),
    proposals(region(20),reason='rgb_analysis_region_budget_exhausted'),
    proposals(*(region(10+25*i,score=.01*i) for i in range(5))),
    replace(proposals(region(20)),selected_index=None,reason='ambiguous_rgb_regions'),
    replace(proposals(region(20),region(50,score=.1)),selected_index=0),
    replace(proposals(region(20),region(50,score=.1)),total_candidate_count=17),
])
def test_empty_budget_truncated_or_inconsistent_selection_rejects(data):
    with pytest.raises(ValueError):select_rgb_interest_set(data,POLICY)


def test_metadata_is_independent_of_candidate_score_order_and_binds_pixels():
    a,b=region(30),region(60,score=.1)
    meta=observation_set_metadata((b,a))
    assert observation_set_digest((a,b))==observation_set_digest((b,a))==digest(meta)
    assert observation_set_metadata_digest(list(meta))==digest(meta)
    with pytest.raises(ValueError):observation_set_metadata_digest(list(reversed(meta)))
    moved=region(61,score=.1)
    assert observation_set_digest((a,moved))!=observation_set_digest((a,b))


@pytest.mark.parametrize('field,value',[
    ('area_px',True),('area_px',0),('mask_sha256','X'*64),
    ('bbox_xywh',[-1,40,12,12]),('image_shape',[0,160]),
    ('centroid_xy',[float('nan'),45.5]),('centroid_xy',[100.,45.5])])
def test_bad_metadata_is_not_an_identity_receipt(field,value):
    row=dict(observation_set_metadata((region(30),))[0]);row[field]=value
    with pytest.raises(ValueError):observation_set_metadata_digest([row])


def test_duplicate_member_or_pixel_metadata_mutation_rejects():
    a=region(30)
    with pytest.raises(ValueError):observation_set_metadata((a,a))
    with pytest.raises(ValueError):observation_set_digest((replace(a,bbox_xywh=(31,40,12,12)),))


def test_three_frames_confirm_unordered_set_without_object_or_motion_authority():
    t=RgbObservationSetTracker();a,b=region(30),region(60,score=.11)
    assert not observe(t,proposals(a,b),0).confirmed
    assert not observe(t,proposals(replace(b,score=0.),replace(a,score=.1)),1).confirmed
    result=observe(t,proposals(b,a),2)
    assert result.confirmed and result.consistent_frames==3 and len(result.regions)==2
    assert result.observation_only and not result.object_identity_confirmed
    assert not result.metric_geometry_authority and not result.motion_authorized


def test_margin_flicker_does_not_drop_a_still_visible_old_member():
    t=RgbObservationSetTracker();a,b=region(30),region(60,score=.11)
    observe(t,proposals(a,b),0)
    observe(t,proposals(a,replace(b,score=.13)),1)
    result=observe(t,proposals(a,replace(b,score=.15)),2)
    assert result.confirmed and len(result.regions)==2


def test_new_competitor_resets_confirmation_without_forgetting_old_members():
    t=RgbObservationSetTracker();a,b=region(30),region(60,score=.1)
    for i in range(3):observe(t,proposals(a),i)
    new=observe(t,proposals(b,a),3)
    assert not new.confirmed and new.consistent_frames==1
    observe(t,proposals(a,replace(b,score=.2)),4)
    assert observe(t,proposals(a,replace(b,score=.2)),5).confirmed


def test_fixed_original_mask_prevents_accumulated_pixel_drift():
    t=RgbObservationSetTracker(max_centroid_shift_fraction=.1)
    observe(t,proposals(region(30)),0)
    assert observe(t,proposals(region(31)),1).consistent_frames==2
    result=observe(t,proposals(region(32)),2)
    # 10/14 IoU passes; a further pixel fails against x=30 even though pairwise
    # overlap with x=32 would still be large.
    assert result.confirmed
    result=observe(t,proposals(region(33)),3)
    assert not result.confirmed and result.consistent_frames==1


def test_joint_motion_is_compared_to_first_pose_not_previous_pose():
    t=RgbObservationSetTracker();p=proposals(region(30))
    observe(t,p,0);observe(t,p,1,joints_rad=[.003]*6)
    assert not observe(t,p,2,joints_rad=[.006]*6).confirmed


def test_mutating_caller_mask_cannot_change_fixed_reference():
    t=RgbObservationSetTracker();a=region(30)
    observe(t,proposals(a),0)
    a.mask[:]=False;a.mask[40:52,33:45]=True
    result=observe(t,proposals(region(33)),1)
    assert result.consistent_frames==1


@pytest.mark.parametrize('change',[
    dict(source_id='changed'),dict(generation=2),dict(stamp_ns=1_100_000_000),
    dict(stamp_ns=3_000_000_000),dict(policy=replace(POLICY,center_distance_weight=5.))])
def test_source_order_gap_and_policy_change_cannot_inherit_confirmation(change):
    t=RgbObservationSetTracker();p=proposals(region(30))
    observe(t,p,0);observe(t,p,1)
    assert not observe(t,p,2,**change).confirmed


def test_malformed_ambiguous_singleton_resets_without_throwing_from_observe():
    t=RgbObservationSetTracker();p=proposals(region(30))
    observe(t,p,0);observe(t,p,1)
    result=observe(t,replace(p,selected_index=None,reason='ambiguous_rgb_regions'),2)
    assert not result.confirmed and not result.regions and result.consistent_frames==0
