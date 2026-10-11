"""A measured side contradiction cannot disappear as unrelated top area grows."""
import numpy as np
import pytest

from alicia_grasp_modes.measured_mask_seeds import _side_seed_colour_evidence


def scene(side_pixels=211, top_pixels=3200):
    shape=(100,100)
    bgr=np.full(shape+(3,),(20,20,180),np.uint8)
    top=np.zeros(shape,bool);top.flat[1000:1000+top_pixels]=True
    side=np.zeros(shape,bool);side.flat[6500:6500+side_pixels]=True
    core=top|side
    support=np.zeros(shape,bool);support[80:]=True
    heights=np.zeros(shape);heights[top]=.02;heights[side]=.006
    bgr[top]=230
    cosine=np.ones(shape);cosine[side]=.6
    return bgr,core,support,heights,.02,cosine,top,side


@pytest.mark.parametrize('side_pixels',[25,211,253])
def test_current_side_evidence_is_independent_of_unrelated_top_area(side_pixels):
    results=[]
    for top_pixels in (250,3200,5000):
        *args,top,side=scene(side_pixels,top_pixels)
        before=[a.copy() if isinstance(a,np.ndarray) else a for a in args]
        result,evidence=_side_seed_colour_evidence(*args)
        assert np.all(result[top])
        assert not np.any(result[side])
        assert evidence['support_colour_demoted_side_pixels']==side_pixels
        results.append(result[side].copy())
        for actual,old in zip(args,before):
            if isinstance(actual,np.ndarray):np.testing.assert_array_equal(actual,old)
    for result in results[1:]:np.testing.assert_array_equal(result,results[0])


@pytest.mark.parametrize('fault',['fewer_than_25','dim_side','weak_chroma','steep_side',
    'same_as_top','different_from_support','missing_support','insufficient_support',
    'insufficient_top','dim_top','upper_surface'])
def test_local_evidence_requirements_still_keep_unproven_or_real_side_seeds(fault):
    *args,top,side=scene(24 if fault=='fewer_than_25' else 211,
                         24 if fault=='insufficient_top' else 3200)
    bgr,core,support,heights,upper,cosine=args
    if fault=='dim_side':bgr[side]=(5,5,5)
    elif fault=='weak_chroma':bgr[side]=(100,100,100)
    elif fault=='steep_side':cosine[side]=.49
    elif fault=='same_as_top':bgr[top]=(20,20,180)
    elif fault=='different_from_support':bgr[side]=(180,20,20)
    elif fault=='missing_support':support[:]=False
    elif fault=='insufficient_support':support[:]=False;support[85:90,40:45]=True
    elif fault=='dim_top':bgr[top]=5
    elif fault=='upper_surface':heights[side]=upper
    result,evidence=_side_seed_colour_evidence(*args)
    np.testing.assert_array_equal(result,core)
    assert evidence['support_colour_demoted_side_pixels']==0


def test_demoted_pixels_are_only_removed_from_hard_foreground_without_background_or_depth_changes():
    *args,top,side=scene()
    bgr,core,support,heights,upper,cosine=args
    before=[a.copy() for a in (bgr,core,support,heights,cosine)]
    result,evidence=_side_seed_colour_evidence(*args)
    assert np.all((core&~result)==side)
    assert not np.any(support[side])
    assert np.all(result[top])
    assert evidence['support_colour_candidate_demotions']==211
    assert evidence['support_colour_demotion_minimum_pixels']==25
    for actual,old in zip((bgr,core,support,heights,cosine),before):
        np.testing.assert_array_equal(actual,old)


def test_scattered_contradictions_cannot_be_summed_to_make_one_qualified_region():
    *args,top,side=scene(0)
    bgr,core,support,heights,upper,cosine=args
    side[65,10:34]=True;side[68,40:58]=True;side[72,80]=True
    core|=side;heights[side]=.006;cosine[side]=.6
    result,evidence=_side_seed_colour_evidence(*args)
    assert evidence['support_colour_candidate_demotions']==43
    assert evidence['support_colour_qualified_regions']==0
    np.testing.assert_array_equal(result,core)


def test_qualified_region_does_not_demote_neighbouring_disconnected_small_regions():
    *args,top,side=scene(0)
    bgr,core,support,heights,upper,cosine=args
    side[65,10:35]=True;side[68,40:53]=True;side[72,80]=True
    core|=side;heights[side]=.006;cosine[side]=.6
    result,evidence=_side_seed_colour_evidence(*args)
    assert not np.any(result[65,10:35])
    assert np.all(result[68,40:53]) and result[72,80]
    assert evidence['support_colour_candidate_demotions']==39
    assert evidence['support_colour_demoted_side_pixels']==25
    assert evidence['support_colour_qualified_regions']==1


def test_original_strong_relative_evidence_keeps_its_existing_pixel_decision():
    *args,top,side=scene(0,250)
    bgr,core,support,heights,upper,cosine=args
    side[65,10:34]=True;side[68,40:56]=True
    core|=side;heights[side]=.006;cosine[side]=.6
    result,evidence=_side_seed_colour_evidence(*args)
    assert not np.any(result[side])
    assert np.all(result[top])
    assert evidence['support_colour_demoted_side_pixels']==40
    assert evidence['support_colour_demotion_evidence']=='established_relative_evidence'
