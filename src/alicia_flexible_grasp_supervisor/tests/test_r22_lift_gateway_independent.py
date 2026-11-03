"""Mapped lift cache/lease flow with explicit synthetic geometry and transport.

The historical path shape is a numeric fixture, not a newly executed lift.
No ROS initialization, publishers, services or devices are used.
"""
from copy import deepcopy
import json
from types import SimpleNamespace as NS
import numpy as np
import pytest
import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Header

from test_lift_linear_gateway import case as original_case,req
from test_r22_typed_client_independent import Peer
from alicia_flexible_grasp.robot import lift_linear_gateway as module
from alicia_flexible_grasp.robot import mapped_contact_execution as mapped_module
from alicia_flexible_grasp.robot.physical_reference_client import PhysicalReferenceLease
from alicia_flexible_grasp.robot.physical_reference_contract import message_bytes
from alicia_flexible_grasp.robot.observation_path_guard import FrozenObservationScene,wire_digest
from alicia_flexible_grasp.robot.stationary_following import SDK_QUANTUM_RAD as Q
from alicia_flexible_grasp.robot.endpoint_correction import sdk_counts


def case(monkeypatch):
    node,planner,plan,sample,params,_=original_case(monkeypatch,capabilities=False)
    params['/grasp'][mapped_module.ENABLED_KEY]=True
    params['/grasp_mode/selection'].update(strategy='observe_return',generation=7)
    for n in module.ARM_NAMES:params['/alicia_controller/constraints'][n]['trajectory']=.09
    end=min(sample['sdk_stamp_ns'],sample['accepted_stamp_ns'])
    sample.update(stationary_window_start_ns=end-300_000_000,stationary_window_end_ns=end,
        sdk_sample_count=4,accepted_sample_count=4)
    clock=NS(ros=max(sample['sdk_stamp_ns'],sample['accepted_stamp_ns'])+50_000_000,mono=100.)
    clock.now=lambda:clock.mono;clock.ros_now=lambda:clock.ros
    monkeypatch.setattr(module.time,'monotonic',clock.now)
    monkeypatch.setattr(rospy,'get_time',lambda:clock.ros*1e-9)
    monkeypatch.setattr(rospy.Time,'now',staticmethod(lambda:rospy.Time(clock.ros//10**9,clock.ros%10**9)))
    scene=FrozenObservationScene.from_contact_plan(plan)
    node._observe_return_origin=dict(selection=deepcopy(params['/grasp_mode/selection']),epoch=sample['epoch_ns'],
        revocation=1,model=params['/robot_description'],config=deepcopy(params['/grasp']),scene=scene,
        identity='explicit-synthetic-lift-session',deadline=300.,
        view_session={'session_id':'synthetic-lift','deadline_ns':clock.ros+200_000_000_000})
    node._validated_observe_return_view_session=lambda origin,session:session
    node._physical_reference_lease=None
    node._trajectory_controller_state=NS(header=Header(stamp=rospy.Time.now()))
    def retime(tr):
        tr=deepcopy(tr)
        for p in tr.joint_trajectory.points:
            p.time_from_start*=4
            p.velocities=[v/4 for v in p.velocities];p.accelerations=[a/16 for a in p.accelerations]
        return tr
    planner._retime_cartesian_plan=retime
    def row(tr,frozen,fk,opening,hold):
        return dict(kind='conditional_continuous_lift_geometry',trajectory_sha256=wire_digest(tr),
            scene_sha256=frozen.scene.digest,model_sha256=fk.model_sha256,plan_id=frozen.scene.plan_id,
            accepted_opening_m=opening,stationary_world_target_removed=False,attachment_confirmed=False,
            bridge='stationary_supplied_hold_to_first_positive_time_point',bridge_start_rad=list(hold.positions),
            whole_arm_collision_checked=False,hardware_response_verified=False,motion_authorized=False,
            grasp_success=False,missing_rigid_tool_body_models=['camera','mount'],complete_tool_body_coverage=False)
    def pair(nominal,shifted,plan,frozen,fk,sample,opening,gripper,bodies,**kw):
        return dict(nominal=row(nominal,frozen,fk,opening,NS(positions=frozen.measured)),
            shifted_with_sdk_bridge=row(shifted,frozen,fk,opening,NS(positions=frozen.sdk)))
    monkeypatch.setattr(module,'validate_lift_pair',pair)
    monkeypatch.setattr(module,'validate_lift_trajectory',
        lambda tr,frozen,fk,gripper,hold,bodies,**kw:row(tr,frozen,fk,frozen.opening,hold))
    old_provider=node._closed_lift_contract_validator.side_effect
    controls=NS(fail_release=False,bad_physical_bridge=False,submissions=[],leases=[],events=[])
    def provider(**kw):
        a=old_provider(**kw)
        if kw.get('reference_domain')=='physical':
            a.update(reference_domain='physical',includes_sdk_bridge=False,includes_controller_bridge=True,
                controller_hold_positions_rad=list(kw['sdk_hold'].positions))
            if controls.bad_physical_bridge:a['controller_hold_positions_rad'][1]+=.001
        return a
    node._closed_lift_contract_validator.side_effect=provider
    def connect(contract,**kw):
        driver=Peer('driver',clock,controls.events);hardware=Peer('hardware',clock,controls.events)
        lease=PhysicalReferenceLease(contract,request_factory=lambda:NS(action_goal=None),
            driver_call=driver,hardware_call=hardware,clock=clock.now,ros_now_ns=clock.ros_now,**kw)
        driver.lease=hardware.lease=lease;lease.test_peers=(driver,hardware)
        controls.leases.append(lease);return lease
    monkeypatch.setattr(mapped_module.PhysicalReferenceLease,'connect',connect)
    def prepare(lease):
        node._physical_reference_lease=lease;lease.prepare();lease.commit()
    node._prepare_physical_reference_lease=prepare
    def release(lease):
        lease.release_prepare()
        if controls.fail_release:raise ValueError('synthetic same-word release ACK missing')
        lease.release_commit();node._physical_reference_lease=None
    node._release_physical_reference_lease=release
    def reference(names,model,domain):
        positions=np.asarray(sample['sdk_positions_rad'])-np.asarray(model.bias_rad)
        return NS(positions=positions.tolist(),velocities=[0.]*6,accelerations=[0.]*6)
    node._observation_reference_for_domain=reference
    def execute(*args,**kw):
        assert planner.observation_execution_monitor is None
        wire=planner.observation_trajectory_schedule(deepcopy(planner._last_pose_plan['plan']))
        audit=planner.observation_path_validator(wire)
        hold=node._observation_controller_hold(module.ARM_NAMES)
        submission=planner.observation_physical_reference_provider(wire,audit,hold)
        controls.submissions.append(submission)
        kw['before_execute']();planner.observation_submission_revalidate(wire,audit)
        envelope=FollowJointTrajectoryActionGoal(header=Header(stamp=rospy.Time.now()),
            goal_id=GoalID(id='synthetic-lift-goal',stamp=rospy.Time.now()),goal=submission.action_goal())
        submission.arm(envelope)
        # Explicit synthetic successful response, never a historical outcome.
        clock.mono+=2.+wire.joint_trajectory.points[-1].time_from_start.to_sec()+.4
        clock.ros=envelope.goal.trajectory.header.stamp.to_nsec()+wire.joint_trajectory.points[-1].time_from_start.to_nsec()+400_000_000
        words=sdk_counts(wire.joint_trajectory.points[-1].positions).tolist()
        sample['sdk_positions_rad']=((np.asarray(words)-2048)*Q).tolist()
        sample['accepted_positions_rad']=list(submission.contract.physical_plan().joint_trajectory.points[-1].positions)
        stamp=clock.ros-1_000_000
        sample.update(sdk_stamp_ns=stamp,accepted_stamp_ns=stamp,opening_stamp_ns=stamp,
            stationary_window_start_ns=stamp-300_000_000,stationary_window_end_ns=stamp)
        for peer in submission.lease.test_peers:
            peer.phase=submission.lease.EXECUTING;peer.goal_terminal=True
            for op in (0,4,5,6):peer.overrides[op]={'sdk_counts':words}
        submission.confirm_result(3,NS(error_code=0))
        return True,'explicit synthetic action completion'
    planner._execute_cached_pose_plan.side_effect=execute
    return NS(node=node,planner=planner,plan=plan,sample=sample,params=params,clock=clock,controls=controls)


def test_r22_lift_check_execute_reuses_exact_episode_bias_and_clocks_and_releases(monkeypatch):
    c=case(monkeypatch);result=c.node.handle_contact_lift(req(c.plan))
    assert result.success,result.message
    cached=c.node._lift_linear_cache;e=cached['context']['physical_reference_episode']
    old=(id(e),e.model.digest,e.local_deadline,e.outer_deadline,e.local_ros_deadline)
    assert not c.controls.leases and not e.finished
    result=c.node.handle_contact_lift(req(c.plan,True));assert result.success,result.message
    evidence=json.loads(result.message)['physical_reference']
    assert evidence['release_verified'] and evidence['episode_success']
    assert (id(e),e.model.digest,e.local_deadline,e.outer_deadline,e.local_ros_deadline)==old
    assert len(c.controls.submissions)==1 and c.controls.leases[0].released
    assert c.node._physical_reference_lease is None and c.node._lift_linear_cache is None
    assert not c.node.handle_contact_lift(req(c.plan,True)).success


@pytest.mark.parametrize('fault',['expired','source_changed','replaced_world','physical_bridge','release'])
def test_r22_lift_mapped_cache_or_release_failure_cannot_be_success(monkeypatch,fault):
    c=case(monkeypatch);r=c.node.handle_contact_lift(req(c.plan));assert r.success,r.message
    e=c.node._lift_linear_cache['context']['physical_reference_episode'];old=e.model.digest
    if fault=='expired':c.clock.mono=e.local_deadline+.001
    elif fault=='source_changed':c.sample['accepted_positions_rad'][1]+=.001
    elif fault=='replaced_world':c.node._closed_lift_contract_validator.scene_digest.return_value='changed-scene'
    elif fault=='physical_bridge':c.controls.bad_physical_bridge=True
    else:c.controls.fail_release=True
    result=c.node.handle_contact_lift(req(c.plan,True))
    assert not result.success,result.message
    assert e.model.digest==old and c.node._lift_linear_cache is None
    assert not c.node.handle_contact_lift(req(c.plan,True)).success
    if fault=='release':
        assert c.controls.leases and not c.controls.leases[0].released
        assert c.node._physical_reference_lease is c.controls.leases[0] and e.chain['failed']
    elif fault in ('expired','source_changed','physical_bridge'):
        assert not c.controls.leases


def test_r22_second_lift_check_cannot_reset_existing_episode_lifetime(monkeypatch):
    c=case(monkeypatch);r=c.node.handle_contact_lift(req(c.plan));assert r.success,r.message
    first=c.node._lift_linear_cache['context']['physical_reference_episode']
    result=c.node.handle_contact_lift(req(c.plan))
    assert not result.success and first.failed and first.chain['failed']
    assert not c.controls.leases


def test_r22_lift_mismatched_execute_plan_closes_the_cached_episode(monkeypatch):
    c=case(monkeypatch);r=c.node.handle_contact_lift(req(c.plan));assert r.success,r.message
    first=c.node._lift_linear_cache['context']['physical_reference_episode']
    changed=req(c.plan,True);changed.plan.poses[3].position.z+=.001
    result=c.node.handle_contact_lift(changed)
    assert not result.success and first.failed and first.chain['failed']
    assert c.node._lift_linear_cache is None and not c.controls.leases


@pytest.mark.parametrize('failed',[False,True])
def test_r22_lift_temporarily_clears_and_then_restores_previous_phase_monitor(monkeypatch,failed):
    c=case(monkeypatch);r=c.node.handle_contact_lift(req(c.plan));assert r.success,r.message
    def old_monitor():raise AssertionError('Previous contact monitor leaked into lift')
    c.planner.observation_execution_monitor=old_monitor
    c.controls.fail_release=failed
    result=c.node.handle_contact_lift(req(c.plan,True))
    assert result.success is not failed
    assert c.planner.observation_execution_monitor is old_monitor
