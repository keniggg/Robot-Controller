"""A small segmentation box must not constrain the observed support footprint."""
from types import SimpleNamespace as NS
import numpy as np
import pytest
from alicia_flexible_grasp.vision.object_geometry import (
    _support_context_mask, _fit_support_plane, deproject_depth,
)


def data():
    D=np.full((240,320),3000,np.uint16);M=np.zeros(D.shape,bool)
    M[112:128,152:168]=True
    K=NS(width=320,height=240,fx=400.,fy=400.,cx=160.,cy=120.)
    return D,M,K,(150,110,20,20)


def test_full_uncertain_box_excluded_and_physical_footprint_observed():
    D,M,K,bbox=data();M[115:119,155:159]=False
    mask=_support_context_mask(D,M,bbox,K,.0001,.3,.12)
    assert not mask[110:130,150:170].any()
    vv,uu=np.nonzero(mask)
    assert uu.max()-uu.min()+1>=160 and vv.max()-vv.min()+1>=160
    assert mask[100,100] and not mask[0,0]
    assert D.min()==3000 and D.max()==3000


def test_footprint_recovers_plane_when_small_local_context_is_biased():
    D,M,K,bbox=data();v,u=np.indices(D.shape)
    patch=(abs(u-160)<18)&(abs(v-120)<18)
    D[patch]=(3000+20*(u[patch]-160)).astype(np.uint16)
    results=[]
    for span in (0.,.12):
        mask=_support_context_mask(D,M,bbox,K,.0001,.3,span)
        pts,_=deproject_depth(D,mask,K,.0001)
        n,d,ratio=_fit_support_plane(pts,.0035,200)
        if n[2]<0:n=-n;d=-d
        results.append((n,d,ratio))
    assert abs(results[0][0][0])>.2
    assert abs(results[1][0][0])<.005
    assert abs(results[1][1]+.3)<.0005
    assert results[1][2]>.9


def test_clipped_context_never_wraps_to_other_image_edge():
    D,M,K,_=data();bbox=(1,1,20,20)
    mask=_support_context_mask(D,M,bbox,K,.0001,.3,.12)
    assert mask[:90,:90].any() and not mask[150:,:].any() and not mask[:,200:].any()


@pytest.mark.parametrize('span',[True,-.1,float('nan'),float('inf')])
def test_invalid_physical_span_fails(span):
    D,M,K,bbox=data()
    with pytest.raises(ValueError,match='support minimum span'):
        _support_context_mask(D,M,bbox,K,.0001,.3,span)


def test_missing_range_cannot_invent_context_scale():
    D,M,K,bbox=data();D[110:130,150:170]=0
    with pytest.raises(ValueError,match='measured target-range'):
        _support_context_mask(D,M,bbox,K,.0001,.3,.12)
