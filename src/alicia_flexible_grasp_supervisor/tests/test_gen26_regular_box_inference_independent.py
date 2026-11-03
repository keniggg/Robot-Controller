"""Independent calibrated ray scenes: inferred sides never become measured data.

The box top and support depths are rendered analytically. GeometryEstimate is
an explicit upstream double with actual deprojected top samples (no side data).
The production qualifier and evidence parser are exercised unchanged.
"""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace as NS
import hashlib
import json

import numpy as np
import pytest

from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.vision.regular_box_inference import (
    qualify_regular_box_top, RegularBoxInference, transform_inferred_box)


def ray_scene(*, tilt=0., width=.030, length=.065):
    h,w=240,320
    k=NS(width=w,height=h,fx=600.,fy=600.,cx=(w-1)/2.,cy=(h-1)/2.)
    ang=np.radians(tilt)
    Ry=np.array([[np.cos(ang),0,np.sin(ang)],[0,1,0],[-np.sin(ang),0,np.cos(ang)]])
    T=np.eye(4); T[:3,:3]=Ry@np.diag([1.,-1.,-1.]); T[:3,3]=[.35*np.sin(ang),0,.35*np.cos(ang)]
    yy,xx=np.mgrid[:h,:w]
    rays=np.stack(((xx-k.cx)/k.fx,(yy-k.cy)/k.fy,np.ones((h,w))),axis=-1)@T[:3,:3].T
    origin=T[:3,3]
    support_t=-origin[2]/rays[:,:,2]
    top_t=(.025-origin[2])/rays[:,:,2]
    top=origin+rays*top_t[:,:,None]
    yaw=np.radians(21.)
    axes=np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1.]])
    local=top@axes
    mask=(np.abs(local[:,:,0])<=length/2)&(np.abs(local[:,:,1])<=width/2)
    depth=np.rint(np.where(mask,top_t,support_t)/.0001).astype(np.uint16)
    measured=origin+rays*(depth*.0001)[:,:,None]
    geom=GeometryEstimate(ok=True,failure_code='',failure_reason='',center_base=np.array([0,0,.0125]),
        axes_base=axes,size_xyz_m=np.array([length,width,.025]),support_normal_base=np.array([0.,0.,1.]),
        support_offset_m=0.,support_inlier_ratio=1.,object_points_base=measured[mask],source_mode='instance_mask')
    return NS(depth=depth,mask=mask.astype(np.uint8)*255,K=k,T=T,geometry=geom,
        identity=TargetTrackIdentity(1,'g4-t1'),stamp=123_456_789_012,top=top,local=local)


def qualify(s,**changes):
    kw=dict(depth_raw=s.depth,object_mask=s.mask,camera_intrinsics=s.K,depth_scale_m=.0001,
        T_base_camera=s.T,geometry=s.geometry,source_identity=s.identity,
        source_stamp_ns=s.stamp,source_mode='instance_mask')
    kw.update(changes)
    return qualify_regular_box_top(**kw)


def assert_unqualified(result):
    assert not result.ok, result.audit
    assert result.model is None
    assert result.audit['measured_side_surfaces'] is False


@pytest.mark.parametrize('tilt',[0.,28.,45.])
def test_complete_metric_rectangle_with_only_top_depth_qualifies_under_perspective(tilt):
    s=ray_scene(tilt=tilt); original=s.geometry.object_points_base.copy()
    r=qualify(s)
    assert r.ok,(r.code,r.audit)
    assert isinstance(r.model,RegularBoxInference)
    np.testing.assert_allclose(sorted(r.model.size_xyz_m[:2]),[.030,.065],atol=.0015)
    np.testing.assert_allclose(r.model.center_base,[0,0,.0125],atol=.001)
    assert np.ptp(original[:,2])<.0002  # no measured side faces in this fixture
    np.testing.assert_array_equal(s.geometry.object_points_base,original)
    assert r.audit['measured_side_surfaces'] is False
    assert r.model.assumption=='orthogonal_vertical_prism_on_verified_support'
    assert r.model.qualification['uncertainty_semantics']=='empirical_budget_not_certified_error_bound'
    assert r.model.source['depth_sha256']==hashlib.sha256(s.depth.tobytes()).hexdigest()
    assert r.model.source['mask_sha256']==hashlib.sha256(s.mask.tobytes()).hexdigest()


def test_perfect_rectangle_artificially_cut_from_half_box_does_not_qualify():
    s=ray_scene(); mask=s.mask.copy(); mask[s.local[:,:,0]>0]=0
    r=qualify(s,object_mask=mask)
    assert_unqualified(r)
    assert r.code in ('BOX_TOP_CONTOUR_INCOMPLETE','BOX_MEASURED_GEOMETRY_CONFLICT')


@pytest.mark.parametrize('defect',['corner','l_shape','interior_hole','one_edge_missing_depth'])
def test_missing_top_evidence_requires_view_instead_of_inventing_corners(defect):
    s=ray_scene(); d=s.depth.copy(); mask=s.mask.copy(); x,y=s.local[:,:,0],s.local[:,:,1]
    if defect=='corner': cut=(x>.015)&(y>.003)
    elif defect=='l_shape':cut=(x>0)&(y>0)
    elif defect=='interior_hole':cut=(abs(x)<.008)&(abs(y)<.006)
    else:cut=(y>.009)
    if defect=='one_edge_missing_depth':d[cut&(mask>0)]=0
    else:mask[cut]=0
    assert_unqualified(qualify(s,depth_raw=d,object_mask=mask))


def test_rectangle_beside_higher_occluder_cannot_prove_whole_box():
    s=ray_scene(); mask=s.mask.copy(); cut=s.local[:,:,0]>0
    mask[cut]=0
    d=s.depth.copy(); d[cut]=np.maximum(1,d[cut].astype(int)-150).astype(np.uint16)
    assert_unqualified(qualify(s,depth_raw=d,object_mask=mask))


def test_large_unresolved_depth_noise_does_not_become_confident_top():
    s=ray_scene(); rng=np.random.RandomState(281)
    d=s.depth.copy(); valid=s.mask>0
    d[valid]=(d[valid].astype(int)+rng.randint(-80,81,np.count_nonzero(valid))).astype(np.uint16)
    assert_unqualified(qualify(s,depth_raw=d))


def test_observed_taper_or_wide_side_conflicts_with_regular_prism_inference():
    s=ray_scene()
    measured=np.vstack((s.geometry.object_points_base,np.array([[.06,0,.008]])))
    r=qualify(s,geometry=replace(s.geometry,object_points_base=measured))
    assert_unqualified(r)
    assert r.code=='BOX_MEASURED_GEOMETRY_CONFLICT'


@pytest.mark.parametrize('fault',['bbox_source','edge_clipped','weak_support','bad_tf','boolean_stamp'])
def test_invalid_or_incomplete_source_cannot_qualify(fault):
    s=ray_scene(); kw={}
    if fault=='bbox_source':kw['source_mode']='bbox_depth'
    elif fault=='edge_clipped':
        mask=s.mask.copy();mask[0,150:170]=255;kw['object_mask']=mask
    elif fault=='weak_support':kw['geometry']=replace(s.geometry,support_inlier_ratio=.1)
    elif fault=='bad_tf':
        T=s.T.copy();T[0,0]*=1.1;kw['T_base_camera']=T
    else:kw['source_stamp_ns']=True
    assert_unqualified(qualify(s,**kw))


def test_model_owns_immutable_provenance_and_registered_transform_without_faking_points():
    s=ray_scene(); r=qualify(s); assert r.ok,r.audit
    model=r.model; original=model.to_evidence_dict()
    copy=model.source;copy['stamp_ns']+=1
    assert model.to_evidence_dict()==original
    with pytest.raises(ValueError):model.center_base.setflags(write=True)
    T=np.eye(4);T[:3,3]=[.01,-.02,.03]
    registered=transform_inferred_box(model,T)
    np.testing.assert_allclose(registered.center_base,model.center_base+T[:3,3])
    assert registered.source==model.source
    assert registered.model_sha256!=model.model_sha256
    assert model.to_evidence_dict()==original


@pytest.mark.parametrize('field,value',[('rectangle_fill_ratio',.7),('corner_coverage',[1.,1.,1.,0.]),
    ('maximum_internal_hole_fraction',.2),('outside_top_continuation_fraction',[.8,0.,0.,0.]),
    ('maximum_observed_model_excess_m',.005),
    ('uncertainty_semantics','certified_bound')])
def test_even_rehashed_model_cannot_claim_failed_completeness_as_qualified(field,value):
    s=ray_scene(); r=qualify(s); assert r.ok,r.audit
    d=r.model.to_evidence_dict();d['qualification'][field]=value
    body={k:v for k,v in d.items() if k!='model_sha256'}
    d['model_sha256']=hashlib.sha256(json.dumps(body,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
    with pytest.raises(ValueError):RegularBoxInference.from_evidence_dict(d)


def rgb_scene():
    import cv2
    s=ray_scene()
    # The segmentation silhouette includes a narrow peripheral skirt/side;
    # only its real top contributes to the measured upper-plane samples.
    top=s.mask>0
    s.mask=cv2.dilate(s.mask,np.ones((5,5),np.uint8))
    color=np.zeros(s.depth.shape+(3,),np.uint8)
    color[top]=[70,140,220]
    return s,color


def test_same_frame_rgb_edges_are_still_supported_by_real_top_and_external_depth():
    s,color=rgb_scene();r=qualify(s,color_bgr=color)
    assert r.ok,(r.code,r.audit)
    assert r.model.qualification['contour_source']=='rgb_edges_on_measured_top_plane'
    assert r.model.source['color_sha256']==hashlib.sha256(color.tobytes()).hexdigest()
    np.testing.assert_allclose(sorted(r.model.size_xyz_m[:2]),[.030,.065],atol=.0015)


def test_printed_inner_rectangle_does_not_shrink_real_box_to_fit_aperture():
    import cv2
    s,color=rgb_scene()
    inner=(abs(s.local[:,:,0])<.021)&(abs(s.local[:,:,1])<.009)
    border=inner&~(cv2.erode(inner.astype(np.uint8),np.ones((3,3),np.uint8))>0)
    color[border]=0
    r=qualify(s,color_bgr=color)
    assert r.ok,(r.code,r.audit)
    np.testing.assert_allclose(sorted(r.model.size_xyz_m[:2]),[.030,.065],atol=.0015)


@pytest.mark.parametrize('fault',['cropped_half','depth_hole'])
def test_rgb_outline_cannot_replace_missing_metric_top_evidence(fault):
    s,color=rgb_scene()
    if fault=='cropped_half':s.mask[s.local[:,:,0]>0]=0
    else:s.depth[(abs(s.local[:,:,0])<.01)&(abs(s.local[:,:,1])<.008)]=0
    assert_unqualified(qualify(s,color_bgr=color))


def test_complete_rgb_top_can_restore_narrow_missing_depth_edge_without_inventing_samples():
    s,color=rgb_scene()
    cut=s.local[:,:,1]>.013
    s.depth[cut&(s.mask>0)]=0
    kept=(s.geometry.object_points_base@s.geometry.axes_base)[:,1]<=.013
    s.geometry=replace(s.geometry,object_points_base=s.geometry.object_points_base[kept])
    measured=s.geometry.object_points_base.copy()
    r=qualify(s,color_bgr=color)
    assert r.ok,(r.code,r.audit)
    assert r.model.qualification['contour_source']=='rgb_edges_on_measured_top_plane'
    np.testing.assert_allclose(sorted(r.model.size_xyz_m[:2]),[.030,.065],atol=.0015)
    np.testing.assert_array_equal(s.geometry.object_points_base,measured)


def test_partial_rgb_boundary_inside_missing_depth_band_cannot_understate_aperture():
    s,color=rgb_scene()
    # Three true boundaries plus a printed/ambiguous fourth line inside an
    # unresolved edge band do not establish a smaller box. If retained, the
    # complete missing extent has to remain in its inferred width allowance.
    cut=s.local[:,:,1]>.010
    s.depth[cut&(s.mask>0)]=0
    color[s.local[:,:,1]>.011]=0
    kept=(s.geometry.object_points_base@s.geometry.axes_base)[:,1]<=.010
    s.geometry=replace(s.geometry,object_points_base=s.geometry.object_points_base[kept])
    r=qualify(s,color_bgr=color)
    if r.ok:
        from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry,evaluate_inferred_box_contact
        g=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)
        e=evaluate_inferred_box_contact(r.model,r.model.center_base,r.model.axes_base[:,1],-r.model.support_normal_base,g)
        assert not e.ok or e.inferred_width_m+e.width_uncertainty_m>=.030-1e-6,(r.audit,e)
    else:assert r.model is None
