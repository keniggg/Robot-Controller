"""Read-only planning regressions. No ROS master or actuator calls."""
import base64
from copy import deepcopy
import json
from pathlib import Path
import threading
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
import pytest
import rospy
from moveit_msgs.msg import RobotState, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
from alicia_flexible_grasp_supervisor.srv import CheckContactSequenceRequest
from alicia_flexible_grasp.robot import contact_sequence_precheck as module
from alicia_flexible_grasp.robot.observation_path_guard import (
    FrozenObservationScene, SerialUrdfFk, wire_digest)
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SDK_QUANTUM_RAD as Q
from test_contact_linear_compensation import recorded
from test_motion_gateway_controller_start import MotionGateway


def planner_for_curve():
    return NS(_retime_cartesian_plan=deepcopy,
              _same_joint_path_geometry=lambda a,b: wire_digest(a)==wire_digest(b))


def collision_fixture():
    d=json.loads((Path(__file__).parent/'fixtures/contact_command_collision_20260930.json').read_text())
    plan=Grasp6DPlan();plan.deserialize(base64.b64decode(d['plan_roswire_base64']))
    tr=RobotTrajectory();tr.deserialize(base64.b64decode(d['nominal_roswire_base64']))
    # Original failed retimed curve was not recorded. Retiming here creates an
    # explicit offline reconstruction, not fabricated hardware measurements.
    for n,p in enumerate(tr.joint_trajectory.points):
        p.time_from_start=rospy.Duration(n*3.)
        p.velocities=[0.]*6;p.accelerations=[0.]*6
    return d,plan,tr


def test_recorded_collision_is_rejected_during_candidate_check_without_motion():
    d,plan,tr=collision_fixture()
    fk=SerialUrdfFk(d['robot_description'], ARM_NAMES)
    collision=mock.Mock(return_value=1)
    with pytest.raises(module.ContactPrecheckError, match='predicted_command.*left_finger'):
        module.validate_contact_prediction(tr,fk,d['sample'],plan.poses[2],
            FrozenObservationScene.from_contact_plan(plan), module.contact_precheck_gripper(d['geometry']),
            d['opening_m'],planner_for_curve(),RobotState(),collision,lambda:5.)
    collision.assert_not_called()  # reject even before the whole-arm service


def test_historical_motor_path_still_passes_both_continuous_curves():
    data,fk,plan,tr,sample=recorded(2)
    before=(wire_digest(tr),wire_digest(plan),deepcopy(sample))
    report=module.validate_contact_prediction(tr,fk,sample,plan.poses[2],
        FrozenObservationScene.from_contact_plan(plan),
        module.contact_precheck_gripper(data['gripper_geometry_config']),.04975,
        planner_for_curve(),RobotState(),lambda *a,**k:12,lambda:5.)
    assert report['collision_state_checks']==12
    assert report['nominal_geometry'] and report['predicted_command_geometry']
    assert report['future_following_verified'] is False
    assert report['execution_authorized'] is False
    assert before==(wire_digest(tr),wire_digest(plan),sample)


def test_virtual_start_is_not_mislabeled_as_future_real_feedback():
    _,fk,plan,tr,sample=recorded(2)
    original=deepcopy(sample)
    delta=np.array([.04,-.03,.02,.01,0.,-.01])
    sample['accepted_positions_rad']=(np.array(sample['accepted_positions_rad'])+delta).tolist()
    sample['sdk_positions_rad']=(np.array(sample['sdk_positions_rad'])+delta).tolist()
    shifted,report=module.predict_contact_command_path(tr,fk,sample,plan.poses[2])
    expected=np.array(original['sdk_positions_rad'])-np.array(original['accepted_positions_rad'])
    for a,b in zip(tr.joint_trajectory.points,shifted.joint_trajectory.points):
        np.testing.assert_allclose(np.array(b.positions)-a.positions,expected,atol=1e-14)
    assert 'measured_start_positions_rad' not in report
    assert report['scope']=='constant_current_bias_prediction'


@pytest.mark.parametrize('fault',['model','nonfinite','excessive_following','retiming','deadline','collision'])
def test_prediction_failure_cannot_authorize_contact(fault):
    data,fk,plan,tr,sample=recorded(2)
    planner=planner_for_curve()
    check=lambda *a,**k:1
    remaining=lambda:5.
    if fault=='model':sample['model_sha256']='other'
    if fault=='nonfinite':sample['sdk_positions_rad'][0]=float('nan')
    if fault=='excessive_following':sample['sdk_positions_rad'][0]+=.1
    if fault=='retiming':planner._same_joint_path_geometry=lambda a,b:False
    if fault=='deadline':
        def remaining():raise module.ContactPrecheckError('deadline','MOVEIT_TIMEOUT')
    if fault=='collision':
        def check(*a,**k):raise module.ContactPrecheckError('collision')
    with pytest.raises(ValueError):
        module.validate_contact_prediction(tr,fk,sample,plan.poses[2],
            FrozenObservationScene.from_contact_plan(plan),
            module.contact_precheck_gripper(data['gripper_geometry_config']),.04975,
            planner,RobotState(),check,remaining)


def gateway_case(monkeypatch):
    data,fk,plan,tr,sample=recorded(2)
    sample.update(epoch_ns=1,accepted_gripper_opening_m=.04975)
    params={'/robot_description':data['robot_description'],
            '/grasp_mode/selection':dict(mode='unknown',strategy='observe_return',generation=4,stamp_ns='1'),
            '/grasp':dict(unknown_contact_cartesian_compensation_enabled=True,unknown_lift_after_close_enabled=True),
            '/gripper':dict(open_position_m=.05,open_skip_tolerance_m=.001),
            '/grasp_6d/remote/gripper_geometry':data['gripper_geometry_config']}
    monkeypatch.setattr(module.rospy,'get_time',lambda:100.)
    monkeypatch.setattr(module.rospy,'get_param',lambda key,default=None:deepcopy(params.get(key,default)))
    node=MotionGateway.__new__(MotionGateway)
    node._planner_lock=threading.RLock();node._control_reference_epoch_ns=1
    node._contact_precheck_sample=mock.Mock(side_effect=lambda *a:deepcopy(sample))
    node._contact_command_collision_check=mock.Mock(return_value=10)
    node._contact_precheck_scene_digest=mock.Mock(return_value='unchanged-scene')
    node._unknown_pregrasp_velocity_limits=lambda _:None
    state=RobotState();state.joint_state.name=list(ARM_NAMES)+['right_finger']
    state.joint_state.position=list(sample['accepted_positions_rad'])+[.04975]
    planner=planner_for_curve();planner.robot=NS(get_current_state=lambda:deepcopy(state))
    def sequence(targets,names,linear,**kw):
        assert list(names)==['pregrasp','approach','grasp','lift']
        assert linear==[False,True,True,True]
        kw['stage_validator']('grasp',state,deepcopy(tr),targets[2])
        return True,'','',dict(path_cost=.5,max_delta=.2),'sequence checked'
    planner.check_pose_sequence=mock.Mock(side_effect=sequence)
    node._ensure_planner=lambda:planner
    node._ensure_trajectory_controllers_started=mock.Mock(side_effect=AssertionError('controller side effect'))
    node._synchronize_trajectory_controller_to_feedback=mock.Mock(side_effect=AssertionError('sync side effect'))
    req=CheckContactSequenceRequest(plan=plan,deadline=rospy.Time.from_sec(110.))
    return node,planner,req,params,sample


def test_readonly_service_never_creates_motion_cache_or_starts_controllers(monkeypatch):
    node,planner,req,params,sample=gateway_case(monkeypatch)
    result=node.handle_contact_sequence_precheck(req)
    assert result.success,result.message
    audit=json.loads(result.evidence_json)
    assert audit['plan_id']==req.plan.plan_id and audit['success'] is True
    assert audit['opening_source']=='current_accepted_open_posture'
    assert not hasattr(node,'_contact_linear_cache')
    assert not hasattr(node,'_compensation_visual_context')
    node._ensure_trajectory_controllers_started.assert_not_called()
    node._synchronize_trajectory_controller_to_feedback.assert_not_called()


@pytest.mark.parametrize('change',['sdk','accepted','epoch','selection','geometry','gripper','model'])
def test_changed_evidence_during_planning_rejects_candidate(monkeypatch,change):
    node,planner,req,params,sample=gateway_case(monkeypatch)
    check=planner.check_pose_sequence.side_effect
    def changed(*a,**k):
        result=check(*a,**k)
        if change=='sdk':sample['sdk_positions_rad'][0]+=Q
        if change=='accepted':sample['accepted_positions_rad'][0]+=2*Q
        if change=='epoch':node._control_reference_epoch_ns=2
        if change=='selection':params['/grasp_mode/selection']['generation']=5
        if change=='geometry':params['/grasp_6d/remote/gripper_geometry']['support_clearance_m']=.004
        if change=='gripper':sample['accepted_gripper_opening_m']-=.001
        if change=='model':params['/robot_description']+='\n'
        return result
    planner.check_pose_sequence.side_effect=changed
    result=node.handle_contact_sequence_precheck(req)
    assert not result.success
    assert 'changed' in result.message
    assert not json.loads(result.evidence_json)['execution_authorized']


def test_deadline_before_planning_never_enters_moveit(monkeypatch):
    node,planner,req,_,_=gateway_case(monkeypatch)
    req.deadline=rospy.Time.from_sec(99.)
    result=node.handle_contact_sequence_precheck(req)
    assert not result.success and result.failure_code=='MOVEIT_TIMEOUT'
    planner.check_pose_sequence.assert_not_called()


def test_virtual_sequence_stops_at_rejected_stage_and_discards_execution_cache():
    from test_moveit_planner_pose_feedback import (
        MoveItPlannerPoseFeedbackTest, FakeManipulator, JointPlan, make_pose)
    fixture=MoveItPlannerPoseFeedbackTest()
    manipulator=FakeManipulator()
    planner=fixture.make_planner(manipulator)
    planner.robot=NS(get_current_state=lambda:fixture.robot_state(0.,0.))
    planner._last_pose_plan=object()
    planner._plan_pose_from_start_state=mock.Mock(return_value=(JointPlan([[.1,.1],[.2,.2]]),'planned'))
    planner._plan_cartesian_from_start_state=mock.Mock(return_value=(JointPlan([[.2,.2],[.3,.3]]),'planned'))
    start=fixture.robot_state(.1,.1)
    seen=[]
    def reject(stage,state,path,target):
        seen.append((stage,list(state.joint_state.position)))
        if stage=='grasp':
            raise module.ContactPrecheckError('predicted_command target collision')
    ok,code,stage,metrics,message=planner.check_pose_sequence(
        [make_pose()]*3,['pregrasp','grasp','lift'],[False,True,True],
        stage_validator=reject,start_state_override=start)
    assert not ok and code=='CONTACT_SEQUENCE_PRECHECK_FAILED' and stage=='grasp'
    assert seen==[('pregrasp',[.1,.1]),('grasp',[.2,.2])]
    planner._plan_cartesian_from_start_state.assert_called_once()
    assert planner._last_pose_plan is None
    assert not manipulator.executed_plans and not manipulator.go_calls and not manipulator.stopped
    assert start.joint_state.position==[.1,.1]


@pytest.mark.parametrize('fault', ['none','collision','wrong_plan','late','no_prediction','no_nominal_cache_fallback'])
def test_remote_consumes_bound_precheck_and_preserves_candidate_failure(monkeypatch,fault):
    from test_remote_grasp6d_streaming import remote_node
    _,_,plan,_,_=recorded(2)
    node=remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    node.contact_sequence_precheck_enabled=True
    clock=[100.]
    node._execution_plan_validity_now_sec=lambda:clock[0]
    node._cached_strict_moveit_sequence_evaluation=mock.Mock(side_effect=AssertionError('nominal fallback'))
    evidence=dict(scope='read_only_candidate_constant_bias_prediction',plan_id=plan.plan_id,
        execution_authorized=False,success=True,checked_stage_names=['pregrasp','approach','grasp','lift'],
        contact_prediction=dict(scope='constant_current_bias_prediction',future_following_verified=False,
            execution_authorized=False,nominal_geometry={'ok':True},predicted_command_geometry={'ok':True}))
    response=NS(success=True,failure_code='',message='execution_duration_lower_bound_sec=4 hardware_limiting_joint=Joint2',
                joint_path_cost=.5,joint_max_delta=.2)
    if fault=='wrong_plan':evidence['plan_id']='another'
    if fault=='no_prediction':evidence.pop('contact_prediction')
    if fault=='collision':
        evidence['success']=False;response.success=False
        response.failure_code='CONTACT_SEQUENCE_PRECHECK_FAILED';response.message='command collision'
    response.evidence_json=json.dumps(evidence)
    def call(req):
        assert req.plan.plan_id==plan.plan_id and req.deadline.to_sec()==105.
        if fault=='late':clock[0]=105.
        if fault=='no_nominal_cache_fallback':raise RuntimeError('service unavailable')
        return response
    monkeypatch.setattr(remote_node.rospy,'wait_for_service',lambda *a,**k:None)
    monkeypatch.setattr(remote_node.rospy,'ServiceProxy',lambda *a,**k:call)
    runtime={}
    result,metrics=node._final_contact_sequence_evaluation(plan,[],NS(),runtime,'test',deadline_sec=105.)
    assert result.reachable is (fault=='none')
    if fault=='none':
        assert metrics['execution_duration_lower_bound_sec']==4.
        assert runtime['moveit_execution_stage_policy']['lift_path_verified']
    if fault=='late':assert result.failure_code=='MOVEIT_TIMEOUT'
    if fault=='collision':assert result.failure_code=='CONTACT_SEQUENCE_PRECHECK_FAILED'
    node._cached_strict_moveit_sequence_evaluation.assert_not_called()


@pytest.mark.parametrize('change', ['none','sdk','accepted','epoch','scene','opening'])
def test_prediction_reuse_requires_exact_feedback_and_world_scene(monkeypatch, change):
    node,planner,req,params,sample=gateway_case(monkeypatch)
    first=node.handle_contact_sequence_precheck(req)
    assert first.success,first.message
    if change=='sdk':sample['sdk_positions_rad'][0]+=Q
    if change=='accepted':sample['accepted_positions_rad'][0]+=Q
    if change=='epoch':node._control_reference_epoch_ns=2
    if change=='scene':node._contact_precheck_scene_digest.return_value='changed-scene'
    if change=='opening':sample['accepted_gripper_opening_m']-=.0001
    # All modified values remain stationary in this second request. Reuse is
    # nonetheless forbidden: the expensive validation must run again.
    second=node.handle_contact_sequence_precheck(req)
    assert second.success,second.message
    audit=json.loads(second.evidence_json)
    assert audit['prediction_cache_hit'] is (change=='none')
    assert planner.check_pose_sequence.call_count==(1 if change=='none' else 2)
    assert not hasattr(node,'_contact_linear_cache')


def test_changed_world_during_precheck_cannot_enter_success_cache(monkeypatch):
    node,planner,req,params,sample=gateway_case(monkeypatch)
    node._contact_precheck_scene_digest.side_effect=['before','after']
    response=node.handle_contact_sequence_precheck(req)
    assert not response.success and 'scene changed' in response.message
    assert not getattr(node,'_contact_precheck_cache',{})
