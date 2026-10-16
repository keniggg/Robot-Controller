"""The explicit inferred strategy still uses physical gripper/enclosure gates."""
from itertools import product
import numpy as np
import pytest

from test_gen26_regular_box_inference_independent import ray_scene, qualify
from alicia_flexible_grasp.vision.regular_box_inference import enclose_inferred_box_geometry
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_flexible_grasp.grasp.tabletop_geometry_candidates import (
    TabletopGeometryConfig, generate_tabletop_proposals, materialize_tabletop_candidates)

GRIPPER=GripperGeometry(.050,.002,np.array([.0434,.0286,.060]),
                        np.array([.1175,.155,.0774]),.003)


def proposals(scene, model, max_gap=.050):
    geometry,audit=enclose_inferred_box_geometry(scene.geometry,model)
    output=generate_tabletop_proposals(geometry.object_points_base,geometry.center_base,
        geometry.axes_base,geometry.size_xyz_m,np.zeros(3),np.array([0.,0.,1.]),
        TabletopGeometryConfig(max_inner_gap_m=max_gap),finger_geometry=GRIPPER,
        inferred_box=model)
    return geometry,audit,output


def test_top_only_qualified_box_produces_physical_candidate_without_faking_measured_sides():
    scene=ray_scene();result=qualify(scene);assert result.ok,result.audit
    original=scene.geometry.object_points_base.copy()
    geometry,audit,output=proposals(scene,result.model)
    assert output.ok,output
    np.testing.assert_array_equal(geometry.object_points_base,original)
    assert audit['measured_side_surfaces'] is False
    assert len(output.sampled_angles_deg)==2
    assert len(output.proposals)==1  # long side is physically wider than 50 mm
    candidate=output.proposals[0]
    assert candidate.required_open_width_m<.050
    assert candidate.negative_contact_count==candidate.positive_contact_count==0
    assert candidate.audit['measured_side_point_counts_available'] is False
    assert not any(k.startswith('measured_') and k.endswith('_m') for k in candidate.audit)
    assert not any(k.startswith('bilateral_contact_height') for k in candidate.audit)
    assert candidate.audit['contact_height_bounds_source']=='regular_box_inferred_from_complete_top'
    materialized=materialize_tabletop_candidates(candidate,np.zeros(3),np.array([0.,0.,1.]),GRIPPER)
    assert len(materialized)>0


def test_inferred_enclosure_contains_original_obb_and_box_yaw_budget():
    scene=ray_scene();model=qualify(scene).model
    g,audit,_out=proposals(scene,model)
    for sign in product((-1.,1.),repeat=3):
        old=scene.geometry.center_base+scene.geometry.axes_base@(np.asarray(sign)*scene.geometry.size_xyz_m/2.)
        assert np.all(np.abs((old-g.center_base)@g.axes_base)<=g.size_xyz_m/2.+1e-10)
    for yaw in np.linspace(-model.empirical_yaw_uncertainty_rad,model.empirical_yaw_uncertainty_rad,21):
        R=np.array([[np.cos(yaw),-np.sin(yaw),0.],[np.sin(yaw),np.cos(yaw),0.],[0.,0.,1.]])
        for sign in product((-1.,1.),repeat=3):
            point=model.center_base+model.axes_base@R@(np.asarray(sign)*model.size_xyz_m/2.)
            assert np.all(np.abs((point-g.center_base)@g.axes_base)<=g.size_xyz_m/2.+1e-10)


def test_near_aperture_needs_precision_view_with_explicit_edge_targets():
    scene=ray_scene(width=.043);r=qualify(scene);assert r.ok,r.audit
    _g,_a,out=proposals(scene,r.model)
    assert not out.ok
    assert out.failure_code=='BOX_INFERENCE_PRECISION_VIEW_REQUIRED'
    rejected=[x for x in out.surface_evidence_rejections if x['failure_code']==out.failure_code]
    assert rejected
    request=rejected[0]['targeted_view']
    assert request['nominal_aperture_margin_m']>0
    assert len(request['target_edges_or_corners_base'])==8
    assert request['width_uncertainty_m']>0


def test_nominally_oversized_box_does_not_claim_more_views_make_it_fit():
    scene=ray_scene(width=.054);r=qualify(scene);assert r.ok,r.audit
    _g,_a,out=proposals(scene,r.model)
    assert not out.ok and out.failure_code=='NO_FIT_DIRECTION'
    assert all(v['failure_code']=='NO_FIT_DIRECTION' for v in out.surface_evidence_rejections)


def test_lower_configured_aperture_is_still_honored():
    scene=ray_scene();r=qualify(scene);assert r.ok
    _g,_a,out=proposals(scene,r.model,max_gap=.025)
    assert not out.ok and out.failure_code=='NO_FIT_DIRECTION'


def test_plain_dict_cannot_opt_into_model_contact():
    scene=ray_scene();r=qualify(scene);assert r.ok
    with pytest.raises(ValueError):
        enclose_inferred_box_geometry(scene.geometry,r.model.to_evidence_dict())
