"""Independent lift proof binding; geometry reports are explicitly synthetic."""
from copy import deepcopy
from types import SimpleNamespace as NS
import pytest

from alicia_flexible_grasp.robot.physical_reference_contract import (
    bind_dual_reference,physical_plan_from_wire,physical_reference_from_wire,message_sha256)
from alicia_flexible_grasp.robot.lift_moveit_adapter import KIND,POLICY
from test_contracted_trajectory import example
from test_r22_reference_map_independent import model


def fixture():
    wire,_,cfg=example();m=model()
    for name in wire.joint_trajectory.joint_names:cfg[name]['trajectory']=.09
    wr=NS(positions=tuple(wire.joint_trajectory.points[0].positions),velocities=(0.,)*6,accelerations=(0.,)*6)
    pr=physical_reference_from_wire(wr,m);physical=physical_plan_from_wire(wire,m)
    nominal_sha=message_sha256(physical)
    def row(plan,reference):
        return dict(kind='conditional_continuous_lift_geometry',trajectory_sha256=message_sha256(plan),
            scene_sha256='a'*64,model_sha256=m.model_sha256,plan_id='synthetic-lift-plan',
            accepted_opening_m=.0498,stationary_world_target_removed=False,attachment_confirmed=False,
            bridge='stationary_supplied_hold_to_first_positive_time_point',bridge_start_rad=list(reference.positions),
            whole_arm_collision_checked=False,hardware_response_verified=False,motion_authorized=False,
            grasp_success=False,missing_rigid_tool_body_models=['camera','mount'],complete_tool_body_coverage=False)
    def audit(plan,reference,domain):
        actual='physical_with_controller_bridge' if domain=='physical' else 'shifted_with_sdk_bridge'
        result=dict(kind=KIND,reference_domain=domain,trajectory_sha256=message_sha256(plan),
            nominal_trajectory_sha256=nominal_sha,scene_sha256='a'*64,model_sha256=m.model_sha256,
            plan_id='synthetic-lift-plan',epoch_ns=m.epoch_ns,opening_width_m=.0498,
            stationary_world_target_removed=False,attachment_confirmed=False,
            whole_arm_collision_method='sampled_exact_Noetic_curve_two_request_local_hypotheses',
            includes_sdk_bridge=domain=='wire',hardware_following_swept_volume_certified=False,
            hardware_stopping_swept_volume_certified=False,controller_constraints_snapshot=deepcopy(cfg),
            execution_tracking_contract=dict(policy=POLICY,stop_trajectory_duration_sec=.5),
            gripper_geometry_snapshot={'max_inner_gap_m':.05},body_geometry_snapshot={},
            missing_rigid_tool_body_models=['camera','mount'],complete_tool_body_coverage=False,
            target_obb_tracking_uncertainty_checked=False,joint_limit_uncertainty_checked=False,
            motion_authorized=False,pure_lift_geometry={'nominal':row(physical,pr),actual:row(plan,reference)})
        if domain=='physical':result.update(includes_controller_bridge=True,controller_hold_positions_rad=list(reference.positions))
        else:result['sdk_hold_positions_rad']=list(reference.positions)
        return result
    return dict(wire_plan=wire,physical_plan=physical,wire_reference=wr,physical_reference=pr,
        following_model=m,wire_audit=audit(wire,wr,'wire'),physical_audit=audit(physical,pr,'physical'))


def test_r22_lift_two_complete_domain_proofs_bind_without_claiming_grasp_or_full_tool_coverage():
    f=fixture();c=bind_dual_reference(**f)
    assert c.proof_kind=='closed_lift'
    goal=c.action_goal()
    assert [t.position for t in goal.path_tolerance]==[.09]*6
    assert [t.position for t in goal.goal_tolerance]==[.035]*6
    for side in ('wire_audit','physical_audit'):
        a=f[side]
        assert a['stationary_world_target_removed'] is False and a['attachment_confirmed'] is False
        assert a['missing_rigid_tool_body_models']==['camera','mount']


@pytest.mark.parametrize('side',['wire_audit','physical_audit'])
@pytest.mark.parametrize('fault',['bridge','bridge_values','point_only','pure_claims_whole_arm',
    'hardware_following','world_removed','attachment','wrong_plan','wrong_domain'])
def test_r22_lift_mapped_proof_cannot_swap_bridge_or_expand_existing_geometry_scope(side,fault):
    f=fixture();a=f[side];actual='physical_with_controller_bridge' if side=='physical_audit' else 'shifted_with_sdk_bridge'
    row=a['pure_lift_geometry'][actual]
    if fault=='bridge':row['bridge']='nominal_t0_only'
    elif fault=='bridge_values':row['bridge_start_rad'][1]+=.001
    elif fault=='point_only':a['whole_arm_collision_method']='endpoint_only'
    elif fault=='pure_claims_whole_arm':row['whole_arm_collision_checked']=True
    elif fault=='hardware_following':a['hardware_following_swept_volume_certified']=True
    elif fault=='world_removed':a['stationary_world_target_removed']=True
    elif fault=='attachment':a['attachment_confirmed']=True
    elif fault=='wrong_plan':row['plan_id']='other-plan'
    else:a['reference_domain']='physical' if side=='wire_audit' else 'wire'
    with pytest.raises(ValueError):bind_dual_reference(**f)


@pytest.mark.parametrize('side',['wire_audit','physical_audit'])
@pytest.mark.parametrize('fault',['missing_list','complete_coverage'])
def test_r22_lift_missing_camera_models_cannot_be_relabelled_complete(side,fault):
    f=fixture();a=f[side]
    # Empty body_geometry_snapshot explicitly means neither model was supplied.
    if fault=='missing_list':a['missing_rigid_tool_body_models']=[]
    else:a['complete_tool_body_coverage']=True
    with pytest.raises(ValueError):bind_dual_reference(**f)


def test_r22_lift_domains_cannot_substitute_even_internally_consistent_body_model():
    f=fixture();a=f['physical_audit']
    a['body_geometry_snapshot']={'camera':dict(frame_id='tool0',
        T_tool_box=[[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]],
        size_xyz_m=[.01,.01,.01],source_sha256='c'*64)}
    a['missing_rigid_tool_body_models']=['mount']
    for row in a['pure_lift_geometry'].values():
        row['missing_rigid_tool_body_models']=['mount'];row['body_sources']={'camera':'c'*64}
    with pytest.raises(ValueError,match='CONTEXT_MISMATCH'):bind_dual_reference(**f)


@pytest.mark.parametrize('side',['wire_audit','physical_audit'])
@pytest.mark.parametrize('field',['body_sources','complete_tool_body_coverage'])
def test_r22_lift_pure_nominal_coverage_cannot_exceed_the_supplied_body_models(side,field):
    f=fixture();row=f[side]['pure_lift_geometry']['nominal']
    row[field]={'camera':'c'*64} if field=='body_sources' else True
    with pytest.raises(ValueError):bind_dual_reference(**f)
