"""Independent inferred-contact transport, aperture and execution provenance.

All models come from the real qualifier over a calibrated analytic depth
scene; no hand-written qualified-box payload or synthetic measured side band.
"""
from copy import deepcopy
import hashlib
import io
import json

import numpy as np
import pytest
import rospy
from tf.transformations import quaternion_from_matrix

from alicia_flexible_grasp.grasp.contact_evidence import (
    encode_inferred_contact_evidence,decode_contact_evidence)
from alicia_flexible_grasp.grasp.contact_execution import plan_contact_evaluator
from alicia_flexible_grasp.grasp.gripper_close_contract import fixed_close_contract
from alicia_flexible_grasp.grasp.gripper_geometry import (
    GripperGeometry,evaluate_inferred_box_contact,ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M)
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id,plan_id_matches_content
from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest
from alicia_flexible_grasp.vision.regular_box_inference import enclose_inferred_box_geometry
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
from test_gen26_regular_box_inference_independent import ray_scene,qualify
from test_rich_plan_candidate_source import neutral_refinement_ros_plan
from test_remote_modes import configured


def physical_gripper(maximum=.05):
    return GripperGeometry(maximum,.002,np.array([.0434,.0286,.0600]),
        np.array([.1175,.1550,.0774]),.003)


def contact_fixture(*,width=.030,angle_deg=0.,maximum=.05):
    s=ray_scene(width=width);q=qualify(s);assert q.ok,(q.code,q.audit)
    model=q.model; a=np.radians(angle_deg)
    rotation=np.column_stack((model.axes_base[:,0],-model.axes_base[:,1],-model.axes_base[:,2]))
    spin=np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1.]])
    rotation=rotation@spin
    projected=evaluate_inferred_box_contact(model,model.center_base,rotation[:,1],rotation[:,2],
        physical_gripper(maximum),opening_fit_clearance_each_side_m=.002)
    return s,model,rotation,projected


def inferred_plan():
    s,model,R,e=contact_fixture();assert e.ok,e
    plan=neutral_refinement_ros_plan();plan.model_choice='unknown_tabletop';plan.diagnostic='CONTACT_EXECUTION_PLAN'
    plan.header.stamp=rospy.Time(s.stamp//1_000_000_000,s.stamp%1_000_000_000)
    plan.target_track_id=s.identity.track_id
    plan.object_geometry.header=deepcopy(plan.header)
    plan.object_geometry.target_track_id=plan.target_track_id
    plan.object_geometry.support_normal_base.x,plan.object_geometry.support_normal_base.y,plan.object_geometry.support_normal_base.z=model.support_normal_base
    plan.object_geometry.support_offset_m=model.support_offset_m
    planning,_=enclose_inferred_box_geometry(s.geometry,model)
    plan.object_geometry.pose_base.position.x,plan.object_geometry.pose_base.position.y,plan.object_geometry.pose_base.position.z=planning.center_base
    planning_T=np.eye(4);planning_T[:3,:3]=planning.axes_base
    plan.object_geometry.pose_base.orientation.x,plan.object_geometry.pose_base.orientation.y,plan.object_geometry.pose_base.orientation.z,plan.object_geometry.pose_base.orientation.w=quaternion_from_matrix(planning_T)
    plan.object_geometry.size_xyz_m.x,plan.object_geometry.size_xyz_m.y,plan.object_geometry.size_xyz_m.z=planning.size_xyz_m
    T=np.eye(4);T[:3,:3]=R
    finger_center=np.asarray(model.center_base).copy();finger_center[2]=.034
    T[:3,3]=finger_center-R@ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M
    pose=plan.poses[2]
    pose.position.x,pose.position.y,pose.position.z=T[:3,3]
    pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=quaternion_from_matrix(T)
    opened=e.inferred_width_m+e.width_uncertainty_m+.004
    plan.required_open_width_m=opened
    plan.contact_geometry_evidence=encode_inferred_contact_evidence(
        contact_center_base=model.center_base,contact_height_axis_base=e.contact_height_axis_base,
        contact_height_bounds_m=e.contact_height_bounds_m,minimum_contact_overlap_m=.002,
        grasp_pose=pose,inferred_contact_width_m=e.inferred_width_m,width_uncertainty_m=e.width_uncertainty_m,
        opening_fit_clearance_each_side_m=.002,required_open_width_m=opened,box_model_evidence=e.model_evidence)
    plan.plan_id=compute_plan_id(plan)
    return plan,model,e,T


def rehash_model(d):
    d['model_sha256']=hashlib.sha256(json.dumps({k:v for k,v in d.items() if k!='model_sha256'},
        sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def freeze_mutation(plan,evidence):
    plan.contact_geometry_evidence=json.dumps(evidence,sort_keys=True,separators=(',',':'),allow_nan=False)
    plan.plan_id=compute_plan_id(plan)
    assert plan_id_matches_content(plan)


def test_inferred_sides_remain_explicit_across_ros_transport_and_execution_report():
    plan,model,projected,T=inferred_plan()
    evidence=decode_contact_evidence(plan)
    assert evidence['schema_version']==3
    assert evidence['measurement_source']=='regular_box_model_inference'
    assert 'measured_contact_width_m' not in evidence
    assert not hasattr(projected,'negative_jaw_points') and not hasattr(projected,'positive_jaw_points')
    assert plan.fused_view_count==0
    buffer=io.BytesIO();plan.serialize(buffer)
    received=Grasp6DPlan().deserialize(buffer.getvalue())
    assert decode_contact_evidence(received)==evidence
    assert committed_plan_digest(received)==committed_plan_digest(plan)
    evaluator=plan_contact_evaluator(received,{},received.required_open_width_m)
    report=evaluator(T)
    assert report['ok'],report
    assert report['contact_sides_are_inferred'] is True
    assert report['contact_geometry_source']=='regular_box_model_inference'
    assert report['physical_grip_verified'] is False
    with pytest.raises(ValueError,match='MEASURED_BILATERAL_CONTACT_REQUIRED'):
        plan_contact_evaluator(received,{},received.required_open_width_m,require_measured_bilateral_contact=True)


@pytest.mark.parametrize('fault',['width','uncertainty','bounds','center','pose','measurement_source','fake_measured_count'])
def test_rehashed_contact_cannot_change_qualified_model_projection_or_provenance(fault):
    plan,model,e,T=inferred_plan();d=decode_contact_evidence(plan)
    if fault=='width':d['inferred_contact_width_m']-=.01;d['required_open_width_m']-=.01;plan.required_open_width_m-=.01
    elif fault=='uncertainty':
        plan.required_open_width_m-=d['width_uncertainty_m'];d['required_open_width_m']=plan.required_open_width_m;d['width_uncertainty_m']=0.
    elif fault=='bounds':d['contact_height_bounds_m'][0]-=.01
    elif fault=='center':d['contact_center_base']=(model.center_base+T[:3,1]*.1).tolist()
    elif fault=='pose':plan.poses[2].position.z+=.001
    elif fault=='measurement_source':d['measurement_source']='fused_measured_bilateral_surface'
    else:d['negative_jaw_points']=100
    freeze_mutation(plan,d)
    with pytest.raises(ValueError):decode_contact_evidence(plan)


@pytest.mark.parametrize('fault',['source_stamp','source_track','plan_track','geometry_track','support_normal','support_offset'])
def test_rehashed_model_and_plan_must_still_match_exact_source_and_registered_support(fault):
    plan,model,e,T=inferred_plan();d=decode_contact_evidence(plan)
    m=d['box_model_evidence']
    if fault=='source_stamp':m['source']['stamp_ns']+=1;rehash_model(m)
    elif fault=='source_track':m['source']['target_identity']['track_id']='g4-t2';rehash_model(m)
    elif fault=='plan_track':
        plan.target_track_id='g4-t2';plan.object_geometry.target_track_id='g4-t2'
    elif fault=='geometry_track':plan.object_geometry.target_track_id='g4-t2'
    elif fault=='support_normal':plan.object_geometry.support_normal_base.x=.01
    else:plan.object_geometry.support_offset_m=.005
    if fault=='geometry_track':
        # This stronger mismatch is already rejected by the signed-plan
        # canonicalizer, before a new identifier can even be constructed.
        with pytest.raises(ValueError,match='target track'):compute_plan_id(plan)
        with pytest.raises(ValueError):decode_contact_evidence(plan)
        return
    freeze_mutation(plan,d)
    with pytest.raises(ValueError):decode_contact_evidence(plan)


def test_large_wrist_reorientation_can_use_narrow_dimension_but_cannot_fit_long_dimension():
    s,model,R,narrow=contact_fixture()
    assert narrow.ok,narrow
    wide=evaluate_inferred_box_contact(model,model.center_base,model.axes_base[:,0],R[:,2],physical_gripper())
    assert not wide.ok and wide.code=='NO_FIT_DIRECTION'
    assert narrow.inferred_width_m<.032
    assert narrow.targeted_view['aperture_margin_m']>.002


def test_near_physical_limit_requests_targeted_precision_instead_of_dropping_uncertainty():
    s,model,R,e=contact_fixture(width=.042)
    assert not e.ok and e.code=='BOX_INFERENCE_PRECISION_VIEW_REQUIRED',e
    assert e.targeted_view['nominal_aperture_margin_m']>0.
    assert e.targeted_view['target_edges_or_corners_base']
    assert e.width_uncertainty_m>0.


def test_actual_rotated_projection_and_center_offset_consume_aperture_allowance():
    _,model,R,aligned=contact_fixture(width=.037)
    _,_,_,rotated=contact_fixture(width=.037,angle_deg=7.)
    assert aligned.ok,aligned
    assert rotated.inferred_width_m>aligned.inferred_width_m
    assert not rotated.ok and rotated.code=='BOX_INFERENCE_PRECISION_VIEW_REQUIRED',rotated
    shifted=evaluate_inferred_box_contact(model,model.center_base+R[:,1]*.009,R[:,1],R[:,2],physical_gripper())
    assert not shifted.ok


def test_configured_gap_above_hardware_cannot_relax_physical_width():
    _,_,_,result=contact_fixture(width=.049,maximum=.08)
    assert not result.ok and result.code=='NO_FIT_DIRECTION'
    assert result.targeted_view['physical_opening_m']==.05


def test_candidate_route_rechecks_precision_margin_against_smaller_effective_aperture():
    from alicia_flexible_grasp.grasp.tabletop_geometry_candidates import (
        generate_tabletop_proposals,TabletopGeometryConfig)
    s,model,R,e=contact_fixture()
    upper=e.inferred_width_m+e.width_uncertainty_m+.004
    result=generate_tabletop_proposals(s.geometry.object_points_base,s.geometry.center_base,
        s.geometry.axes_base,s.geometry.size_xyz_m,np.zeros(3),np.array([0.,0.,1.]),
        TabletopGeometryConfig(max_inner_gap_m=upper+.001),finger_geometry=physical_gripper(),inferred_box=model)
    assert not result.ok,result
    assert result.failure_code=='BOX_INFERENCE_PRECISION_VIEW_REQUIRED'


def test_fixed_close_uses_nominal_width_not_inferred_upper_opening_as_contact_proof():
    plan,model,e,T=inferred_plan()
    c=fixed_close_contract(plan,dict(use_compliant_close=False,use_plan_bound_close_position=True,
        close_limit_m=0.,open_position_m=.05,plan_bound_close_preload_m=.002))
    assert c['close_target_inner_gap_m']==pytest.approx(e.inferred_width_m-.002,abs=5e-9)
    assert c['close_target_inner_gap_m']<plan.required_open_width_m-e.width_uncertainty_m


@pytest.mark.parametrize('fault',['shrink','shift','rotate'])
def test_rehashed_final_collision_enclosure_cannot_drop_inferred_box_uncertainty(fault):
    plan,model,e,T=inferred_plan()
    g=plan.object_geometry
    if fault=='shrink':g.size_xyz_m.x*=.8;g.size_xyz_m.y*=.8;g.size_xyz_m.z*=.8
    elif fault=='shift':g.pose_base.position.x+=.1
    else:
        R=np.eye(4);R[:3,:3]=model.axes_base@np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
        g.pose_base.orientation.x,g.pose_base.orientation.y,g.pose_base.orientation.z,g.pose_base.orientation.w=quaternion_from_matrix(R)
    plan.plan_id=compute_plan_id(plan)
    assert plan_id_matches_content(plan)
    with pytest.raises(ValueError):decode_contact_evidence(plan)


def initial_route(configured,monkeypatch,*,width):
    from test_measured_observation_planning import preflight_context
    from alicia_flexible_grasp.vision.multiview_surface import SurfaceView,fused_surface_from_view
    from alicia_flexible_grasp.grasp.tabletop_geometry_candidates import generate_tabletop_proposals
    node,ticket,snapshot,_old_geometry,transform,_,_,guards=preflight_context(configured,monkeypatch)
    s=ray_scene(width=width);s.identity=snapshot.target_identity;s.stamp=snapshot.stamp_ns
    result=qualify(s);assert result.ok,result.audit
    model=result.model
    view=SurfaceView(s.identity,s.stamp,s.geometry.object_points_base,
        s.geometry.support_normal_base,s.geometry.support_offset_m,12)
    node._near_field_fused_surface=fused_surface_from_view(view)
    node._near_field_reference_view=view
    monkeypatch.setattr(node,'_current_inferred_box',lambda *args:model)
    generated=generate_tabletop_proposals(s.geometry.object_points_base,s.geometry.center_base,
        s.geometry.axes_base,s.geometry.size_xyz_m,np.zeros(3),s.geometry.support_normal_base,
        node.tabletop_geometry_config,finger_geometry=node.gripper_geometry,inferred_box=model)
    diagnostics=dict(materialized_count=len(generated.proposals),plan_phase='CONTACT_EXECUTION_PLAN',
        contact_execution_gate_deferred=False,surface_evidence_rejections=deepcopy(generated.surface_evidence_rejections))
    return node,ticket,snapshot,s.geometry,transform,generated,diagnostics,model


def test_actual_initial_contact_route_does_not_request_side_view_when_complete_box_has_valid_direction(configured,monkeypatch):
    node,ticket,snapshot,geometry,T,generated,diagnostics,model=initial_route(configured,monkeypatch,width=.030)
    assert generated.proposals
    assert not node._maybe_prepare_observation_from_contact_geometry(ticket,snapshot,geometry,T,
        generated.proposals,diagnostics)
    assert node._observe_return_phase=='initial_contact'
    assert node._observe_return_trigger_evidence is None


def test_actual_initial_contact_route_targets_precision_when_narrow_side_uncertainty_nearly_exhausts_opening(configured,monkeypatch):
    node,ticket,snapshot,geometry,T,generated,diagnostics,model=initial_route(configured,monkeypatch,width=.042)
    assert not generated.proposals and generated.failure_code=='BOX_INFERENCE_PRECISION_VIEW_REQUIRED'
    assert node._maybe_prepare_observation_from_contact_geometry(ticket,snapshot,geometry,T,(),diagnostics)
    assert node._observe_return_phase=='observation_prefix'
    proof=node._observe_return_trigger_evidence
    assert proof['policy']=='current_request_box_precision_view_preflight'
    assert proof['contact_or_motion_authorized'] is False
    assert 'targeted_precision_views' in proof and 'measured_missing_sides' not in proof
    for target in proof['targeted_precision_views']:
        assert target['model_sha256']==model.model_sha256
        assert target['side_selection_role']=='precision_view_target_not_missing_measurement'
        assert target['measured_side_surfaces'] is False
    assert not any(plan.valid for plan in node.rich_plan_pub.messages)


def test_actual_precision_route_does_not_borrow_other_model_source(configured,monkeypatch):
    node,ticket,snapshot,geometry,T,generated,diagnostics,model=initial_route(configured,monkeypatch,width=.042)
    other=ray_scene(width=.042);other.identity=snapshot.target_identity;other.stamp=snapshot.stamp_ns+1
    other_model=qualify(other).model
    monkeypatch.setattr(node,'_current_inferred_box',lambda *args:other_model)
    assert not node._maybe_prepare_observation_from_contact_geometry(ticket,snapshot,geometry,T,(),diagnostics)
    assert node._observe_return_phase=='initial_contact'
    assert node._observe_return_trigger_evidence is None
