"""Observed background trimming preserves the full required measurement crop."""
import itertools
import numpy as np
import pytest
from alicia_grasp_modes.rgb_instance import _observed_spatial_search_bounds


@pytest.mark.parametrize('seed', range(8))
def test_maximum_area_and_deterministic_tie_match_exhaustive_rectangles(seed):
    rng=np.random.RandomState(seed)
    valid=rng.rand(14,14)>.18
    valid[2:12,2:12]=True
    original=valid.copy()
    expected=[]
    for x0,y0,x1,y1 in itertools.product(range(3),range(3),range(12,15),range(12,15)):
        if np.all(valid[y0:y1,x0:x1]):
            expected.append((-(x1-x0)*(y1-y0),x0,y0,x1,y1))
    expected=min(expected)[1:]
    actual=_observed_spatial_search_bounds(valid,(0,0,14,14),(4,4,10,10),(6,6,8,8))
    assert actual==expected
    assert actual==_observed_spatial_search_bounds(valid,(0,0,14,14),(4,4,10,10),(6,6,8,8))
    np.testing.assert_array_equal(valid,original)


def test_multiple_invalid_edges_are_trimmed_without_clipping_first_core_or_prism():
    valid=np.ones((120,140),bool)
    valid[:3]=False;valid[-2:]=False;valid[:,:5]=False;valid[:,-4:]=False
    assert _observed_spatial_search_bounds(valid,(0,0,140,120),(20,20,120,100),(30,30,110,90))==(5,3,136,118)


def test_first_core_crop_outside_prism_remains_required():
    valid=np.ones((100,140),bool);valid[:,:2]=False
    first=(8,12,132,90);prism=(35,30,85,60)
    bounds=_observed_spatial_search_bounds(valid,(0,0,140,100),first,prism)
    assert bounds==(2,0,140,100)
    assert bounds[0]<=first[0] and bounds[2]>=first[2]


def test_all_observed_desired_bounds_are_unchanged():
    valid=np.ones((100,140),bool)
    assert _observed_spatial_search_bounds(valid,(0,0,140,100),(8,12,132,90),(35,30,85,60))==(0,0,140,100)
