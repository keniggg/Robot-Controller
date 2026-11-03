"""Exercise the real adapter with fake read-only ROS service transports."""
from copy import deepcopy
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
import pytest
from moveit_msgs.msg import RobotState,PlanningScene,AttachedCollisionObject

from alicia_flexible_grasp.robot.lift_moveit_adapter import (
    ClosedLiftMoveItAdapter,bound_lift_action_goal,KIND,POLICY)
from alicia_flexible_grasp.robot.lift_path_continuity import derive_lift_path
from alicia_flexible_grasp.robot.observation_path_guard import wire_digest,ObservationPathError
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES
from test_lift_path_continuity import recorded,point


def inputs():
    data,fk,plan,tr,sample,opening=recorded()
    shifted,frozen,_=derive_lift_path(tr,plan,fk,sample,opening)
    state=RobotState();state.joint_state.name=list(ARM_NAMES)+['right_finger']
    state.joint_state.position=sample['accepted_positions_rad']+[opening]
    old=AttachedCollisionObject();old.object.id='preexisting_attachment'
    state.attached_collision_objects=[old]
    constraints={n:dict(trajectory=.12,goal=.035) for n in ARM_NAMES}
    constraints.update(goal_time=2.,stopped_velocity_tolerance=.01)
    args=dict(nominal=tr,shifted=shifted,frozen=frozen,start_state=state,
        sdk_hold=point(sample['sdk_positions_rad']),bodies=(),gripper=None,
        constraints=constraints,stop_duration=.1)
    return data,args


def adapter_case(check=None,scene=None,**options):
    data,args=inputs();records=[]
    def valid(req):
        records.append(deepcopy(req))
        return check(req) if check else NS(valid=True)
    read=mock.Mock(side_effect=scene or (lambda req:NS(scene=PlanningScene())))
    adapter=ClosedLiftMoveItAdapter(data['robot_description'],valid,read,**options)
    return adapter,args,records,read


def test_real_adapter_checks_both_hypotheses_for_nominal_shifted_and_bridge_without_scene_mutation():
    a,args,requests,scene=adapter_case()
    original=wire_digest(args['start_state']),wire_digest(args['nominal']),wire_digest(args['shifted'])
    audit=a(**args)
    assert audit['kind']==KIND and not audit['hardware_following_swept_volume_certified']
    assert not audit['hardware_stopping_swept_volume_certified']
    assert audit['missing_rigid_tool_body_models']==['camera','mount']
    assert scene.call_count==2 and len(requests)==audit['whole_arm_state_checks']
    assert len(requests)>4*len(args['nominal'].joint_trajectory.points)
    world=[];carried=[]
    for req in requests:
        assert req.group_name=='' and req.robot_state.is_diff
        assert req.robot_state.attached_collision_objects[0].object.id=='preexisting_attachment'
        item=req.robot_state.attached_collision_objects[-1]
        assert item.link_name=='tool0'
        assert item.touch_links==['Link7','Link8']
        assert item.object.operation==item.object.ADD
        assert item.object.primitive_poses[0].orientation.w==1.
        assert req.robot_state.joint_state.position[-1]==args['frozen'].opening
        (world if item.object.header.frame_id=='base_link' else carried).append(item.object)
    assert len(world)==len(carried)
    assert all(str(obj.pose)==str(world[0].pose) for obj in world)
    assert all(str(obj.pose)==str(carried[0].pose) for obj in carried)
    assert world[0].pose.position.x==pytest.approx(args['frozen'].scene.center[0])
    assert carried[0].pose.position.z==pytest.approx(args['frozen'].T_tool_object[2,3])
    assert original==(wire_digest(args['start_state']),wire_digest(args['nominal']),wire_digest(args['shifted']))


def test_request_rejection_does_not_continue_checks_or_mutate_global_scene():
    a,args,requests,scene=adapter_case(check=lambda req:NS(valid=False))
    with pytest.raises(ObservationPathError,match='LIFT_MOVEIT_COLLISION'):a(**args)
    assert len(requests)==1 and scene.call_count==1


def test_model_with_collision_at_request_local_anchor_is_not_accepted():
    data,args=inputs()
    xml=data['robot_description'].replace('name="tool0">','name="tool0"><collision><geometry><sphere radius=".01"/></geometry></collision>')
    with pytest.raises(ObservationPathError,match='TOOL0_FRAME'):
        ClosedLiftMoveItAdapter(xml,None,None)


def test_scene_change_is_detected_without_changing_any_scene():
    scenes=[PlanningScene(),PlanningScene()];scenes[1].name='changed'
    a,args,requests,scene=adapter_case(scene=lambda req:NS(scene=scenes.pop(0)))
    with pytest.raises(ObservationPathError,match='SCENE_CHANGED_DURING_CHECK'):a(**args)
    assert scene.call_count==2


def test_environment_digest_ignores_only_live_joint_feedback_not_attached_geometry():
    scene=PlanningScene()
    a,args,requests,read=adapter_case(scene=lambda req:NS(scene=deepcopy(scene)))
    initial=a.scene_digest()
    scene.robot_state.joint_state.position=[1.,2.]
    assert a.scene_digest()==initial
    obj=AttachedCollisionObject();obj.object.id='new_attachment';scene.robot_state.attached_collision_objects.append(obj)
    assert a.scene_digest()!=initial


def test_true_noetic_spline_overshoot_is_sampled_not_just_waypoint_chords():
    data,args=inputs();tr=deepcopy(args['nominal'])
    q=list(tr.joint_trajectory.points[0].positions)
    tr.joint_trajectory.points=[point(q,0),point(q,1),point(q,2)]
    tr.joint_trajectory.points[1].velocities=[.4,0,0,0,0,0]
    tr.joint_trajectory.points[2].velocities=[-.4,0,0,0,0,0]
    samples=list(ClosedLiftMoveItAdapter._samples(tr,point(q)))
    assert max(row[0][0] for row in samples)>q[0]+.05
    assert all(np.max(abs(b[0]-a[0]))<=.002+1e-12 for a,b in zip(samples,samples[1:]))


def test_state_request_budget_terminates_readonly_work():
    a,args,requests,scene=adapter_case(max_requests=3)
    with pytest.raises(ObservationPathError,match='CHECK_BUDGET'):a(**args)
    assert len(requests)==3


def complete_action_audit(args):
    audit=dict(kind=KIND,trajectory_sha256=wire_digest(args['shifted']),
        nominal_trajectory_sha256=wire_digest(args['nominal']),scene_sha256=args['frozen'].scene.digest,
        model_sha256=args['frozen'].model_sha256,opening_width_m=args['frozen'].opening,
        controller_constraints_snapshot=deepcopy(args['constraints']),
        execution_tracking_contract=dict(policy=POLICY,stop_trajectory_duration_sec=.1),
        stationary_world_target_removed=False,attachment_confirmed=False,
        whole_arm_collision_method='sampled_exact_Noetic_curve_two_request_local_hypotheses',
        includes_sdk_bridge=True,hardware_following_swept_volume_certified=False,
        hardware_stopping_swept_volume_certified=False,sdk_hold_positions_rad=list(args['sdk_hold'].positions))
    audit['pure_lift_geometry']={label:dict(trajectory_sha256=audit[key],scene_sha256=audit['scene_sha256'],
        model_sha256=audit['model_sha256'],accepted_opening_m=audit['opening_width_m'],
        stationary_world_target_removed=False,attachment_confirmed=False)
        for label,key in [('nominal','nominal_trajectory_sha256'),('shifted_with_sdk_bridge','trajectory_sha256')]}
    return audit


def test_action_builder_preserves_original_controller_limits_and_no_geometry_claim():
    _,args=inputs();audit=complete_action_audit(args)
    goal=bound_lift_action_goal(args['shifted'],audit,args['constraints'],.1,reference=args['sdk_hold'])
    assert [t.position for t in goal.path_tolerance]==[.12]*6
    assert [t.position for t in goal.goal_tolerance]==[.035]*6
    assert goal.goal_time_tolerance.to_sec()==2.
    assert wire_digest(goal.trajectory)==wire_digest(args['shifted'].joint_trajectory)


@pytest.mark.parametrize('fault',['trajectory','scene','opening','sdk','following_claim','stopping_claim','attachment','missing_nominal','limits'])
def test_action_builder_refuses_unbound_or_falsely_claimed_proof(fault):
    _,args=inputs();audit=complete_action_audit(args)
    if fault=='trajectory':audit['trajectory_sha256']='wrong'
    if fault=='scene':audit['scene_sha256']='wrong'
    if fault=='opening':audit['opening_width_m']+=.001
    if fault=='sdk':args['sdk_hold'].positions[0]+=.001
    if fault=='following_claim':audit['hardware_following_swept_volume_certified']=True
    if fault=='stopping_claim':audit['hardware_stopping_swept_volume_certified']=True
    if fault=='attachment':audit['attachment_confirmed']=True
    if fault=='missing_nominal':audit['pure_lift_geometry'].pop('nominal')
    if fault=='limits':args['constraints']['Joint2']['trajectory']=-1.
    with pytest.raises(ObservationPathError):
        bound_lift_action_goal(args['shifted'],audit,args['constraints'],.1,reference=args['sdk_hold'])


@pytest.mark.parametrize('fault',['short_positions','long_positions','nan','inf','duplicate','multidof'])
def test_malformed_state_cannot_be_silently_truncated_or_ignored_by_ros_conversion(fault):
    a,args,requests,scene=adapter_case()
    state=args['start_state']
    if fault=='short_positions':state.joint_state.position=state.joint_state.position[:-1]
    if fault=='long_positions':state.joint_state.position.append(0.)
    if fault=='nan':state.joint_state.position[0]=float('nan')
    if fault=='inf':state.joint_state.position[0]=float('inf')
    if fault=='duplicate':state.joint_state.name[-1]=state.joint_state.name[0]
    if fault=='multidof':state.multi_dof_joint_state.joint_names=['unsupported_floating_base']
    with pytest.raises(ObservationPathError,match='START_STATE_INCOMPLETE'):a(**args)
    assert not requests
