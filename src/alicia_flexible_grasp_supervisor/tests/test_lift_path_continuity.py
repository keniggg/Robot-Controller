"""Pure offline lift contracts; no ROS node, parameters, services or publishers."""
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path

import numpy as np
import pytest
import rospy
from genpy.message import fill_message_args
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
from alicia_flexible_grasp.robot.lift_path_continuity import (
    FrozenLiftContext, ToolBodyBox, LiftPathError, derive_lift_path,
    validate_lift_trajectory, validate_lift_pair, occupancy_at,
)
from alicia_flexible_grasp.robot.observation_path_guard import FrozenObservationScene, SerialUrdfFk, wire_digest
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry


def recorded():
    data=json.loads((Path(__file__).parent/'fixtures/lift_following_20260928.json').read_text())
    fk=SerialUrdfFk(data['robot_description'],ARM_NAMES)
    plan=Grasp6DPlan();fill_message_args(plan,[data['plan']])
    tr=RobotTrajectory();fill_message_args(tr.joint_trajectory,[data['trajectory']])
    def ns(s):return s['secs']*10**9+s['nsecs']
    accepted=data['accepted'];sdk=data['sdk']
    positions=dict(zip(accepted['name'],accepted['position']))
    opening=positions['right_finger']
    sample=dict(model_sha256=fk.model_sha256,joint_names=list(ARM_NAMES),
        accepted_positions_rad=[positions[n] for n in ARM_NAMES],
        sdk_positions_rad=[dict(zip(sdk['name'],sdk['position']))[n] for n in ARM_NAMES],
        accepted_opening_m=opening,epoch_ns=ns(data['epoch']['stamp']),
        accepted_stamp_ns=ns(accepted['header']['stamp']),sdk_stamp_ns=ns(sdk['header']['stamp']))
    return data,fk,plan,tr,sample,opening


class TranslationFk:
    names=ARM_NAMES
    model_sha256='f'*64
    limits=np.array([[-2.,2.]]*6)
    def __call__(self,q):
        q=np.asarray(q);T=np.eye(4);T[:3,3]=q[:3]+[0.,0.,.4];return T
    def point_motion_radii(self,r):return np.array([1.,1.,1.,0.,0.,0.])


def gripper():
    return GripperGeometry(.05,.002,np.array([.0434,.0286,.0600]),np.array([.1175,.155,.0774]),.003)


def point(q,t=0.,v=None):
    return JointTrajectoryPoint(positions=list(q),velocities=[0.]*6 if v is None else v,
        accelerations=[0.]*6,time_from_start=rospy.Duration.from_sec(t))


def trajectory(qs):
    tr=RobotTrajectory();tr.joint_trajectory.joint_names=list(ARM_NAMES)
    tr.joint_trajectory.points=[point(q,i) for i,q in enumerate(qs)]
    return tr


def synthetic(center=None):
    fk=TranslationFk();q=np.zeros(6)
    center=np.array([0.,0.,.2]) if center is None else np.array(center)
    scene=FrozenObservationScene('plan','s'*64,100,np.array([0.,0.,1.]),0.,center,np.eye(3),np.array([.01]*3),'track','g'*64)
    T=np.eye(4);T[:3,3]=center-fk(q)[:3,3]
    context=FrozenLiftContext(scene,fk.model_sha256,q,q,.01,10,20,21,T,'existing contact')
    tr=trajectory([q,q+[0,0,.02,0,0,0],q+[0,0,.04,0,0,0]])
    return fk,context,tr


def test_exact_recorded_all_thirteen_points_keep_offset_and_derivatives():
    _,fk,plan,tr,sample,opening=recorded()
    before=(wire_digest(tr),wire_digest(plan))
    shifted,context,audit=derive_lift_path(tr,plan,fk,sample,opening)
    bias=context.sdk-context.measured
    assert len(tr.joint_trajectory.points)==13
    for a,b in zip(tr.joint_trajectory.points,shifted.joint_trajectory.points):
        np.testing.assert_allclose(np.array(b.positions)-a.positions,bias,atol=1e-15)
        assert a.velocities==b.velocities and a.accelerations==b.accelerations
        assert a.time_from_start==b.time_from_start
    np.testing.assert_allclose(shifted.joint_trajectory.points[0].positions,context.sdk,atol=1e-12)
    assert before==(wire_digest(tr),wire_digest(plan))
    assert not audit['attachment_confirmed'] and not audit['motion_authorized']
    assert not audit['stationary_world_target_removed']
    assert opening == sample['accepted_opening_m']


@pytest.mark.parametrize('fault',['first_only','last_only','derivative','time'])
def test_partial_or_retimed_bias_cannot_reuse_pair_certificate(fault):
    _,fk,plan,tr,sample,opening=recorded()
    shifted,context,_=derive_lift_path(tr,plan,fk,sample,opening)
    if fault=='first_only':shifted.joint_trajectory.points[1:]=deepcopy(tr.joint_trajectory.points[1:])
    if fault=='last_only':shifted.joint_trajectory.points[0]=deepcopy(tr.joint_trajectory.points[0])
    if fault=='derivative':shifted.joint_trajectory.points[2].velocities[0]+=.01
    if fault=='time':shifted.joint_trajectory.points[2].time_from_start+=rospy.Duration(.01)
    with pytest.raises(LiftPathError,match='whole lift bias'):
        validate_lift_pair(tr,shifted,plan,context,fk,sample,opening,gripper())


@pytest.mark.parametrize('fault',['epoch','stamp','opening','sdk','actual','model','plan'])
def test_changed_frozen_context_cannot_reuse_certificate(fault):
    _,fk,plan,tr,sample,opening=recorded()
    _,context,_=derive_lift_path(tr,plan,fk,sample,opening)
    if fault=='epoch':sample['epoch_ns']+=1
    if fault=='stamp':sample['accepted_stamp_ns']+=1
    if fault=='opening':opening+=.0001;sample['accepted_opening_m']=opening
    if fault=='sdk':sample['sdk_positions_rad'][0]+=.0001
    if fault=='actual':sample['accepted_positions_rad'][0]+=.0001
    if fault=='model':sample['model_sha256']='x'
    if fault=='plan':plan.object_geometry.size_xyz_m.x+=.001
    with pytest.raises(ValueError):context.assert_binding(plan,fk,sample,opening)


def test_dual_occupancy_keeps_world_geometry_after_hypothetical_lift():
    fk,c,tr=synthetic(); original=c.scene.center.copy()
    first=occupancy_at(c,fk(np.zeros(6)));last=occupancy_at(c,fk([0,0,.04,0,0,0]))
    np.testing.assert_array_equal(first[0][0],last[0][0]);np.testing.assert_array_equal(original,c.scene.center)
    np.testing.assert_allclose(last[1][0]-first[1][0],[0,0,.04])
    report=validate_lift_trajectory(tr,c,fk,gripper(),point(np.zeros(6)))
    assert report['missing_rigid_tool_body_models']==['camera','mount']
    assert not report['complete_tool_body_coverage'] and not report['motion_authorized']
    assert report['segments']==2


def test_supplied_camera_and_mount_are_bound_and_have_no_contact_exemption():
    fk,c,tr=synthetic()
    bodies=[]
    for name,x in [('camera',.13),('mount',.1)]:
        T=np.eye(4);T[:3,3]=[x,0,0]
        bodies.append(ToolBodyBox(name,T,np.array([.02]*3),'a'*64))
    report=validate_lift_trajectory(tr,c,fk,gripper(),point(np.zeros(6)),bodies)
    assert report['complete_tool_body_coverage'] and not report['whole_arm_collision_checked']
    T=np.eye(4);T[:3,3]=c.scene.center-fk(np.zeros(6))[:3,3]
    body=ToolBodyBox('camera',T,np.array([.02]*3),'b'*64)
    with pytest.raises(LiftPathError,match='initial CAD-envelope/target intersection'):
        validate_lift_trajectory(tr,c,fk,gripper(),point(np.zeros(6)),[body])


def test_retained_world_target_catches_lateral_palm_collision_after_start():
    fk,c,tr=synthetic(center=[.16,0,.4])
    tr=trajectory([np.zeros(6),[.16,0,0,0,0,0]])
    with pytest.raises(LiftPathError,match='world object collision'):
        validate_lift_trajectory(tr,c,fk,gripper(),point(np.zeros(6)))


def test_carried_hypothesis_support_violation_even_when_gripper_stays_clear():
    fk,c,tr=synthetic(center=[0,0,.015])
    tr=trajectory([np.zeros(6),[0,0,-.02,0,0,0]])
    with pytest.raises(LiftPathError,match='carried object enters support'):
        validate_lift_trajectory(tr,c,fk,gripper(),point(np.zeros(6)))


def test_bridge_cannot_be_replaced_with_nominal_t0():
    fk,c,tr=synthetic()
    validate_lift_trajectory(tr,c,fk,gripper(),point(np.zeros(6)))
    with pytest.raises(LiftPathError,match='carried object enters support'):
        validate_lift_trajectory(tr,c,fk,gripper(),point([0,0,-.25,0,0,0]))


def test_spline_overshoot_between_safe_endpoints_is_rejected():
    fk,c,tr=synthetic(center=[0,0,.04])
    tr=trajectory([np.zeros(6),np.zeros(6),np.zeros(6)])
    tr.joint_trajectory.points[1].velocities=[0,0,-1.,0,0,0]
    tr.joint_trajectory.points[2].velocities=[0,0,1.,0,0,0]
    with pytest.raises(LiftPathError,match='carried object enters support'):
        validate_lift_trajectory(tr,c,fk,gripper(),point(np.zeros(6)))


@pytest.mark.parametrize('fault',['stamp','names','time','nan','moving_hold','opening','model'])
def test_invalid_curve_context_rejected(fault):
    fk,c,tr=synthetic();hold=point(np.zeros(6))
    if fault=='stamp':tr.joint_trajectory.header.stamp=rospy.Time(1)
    if fault=='names':tr.joint_trajectory.joint_names.reverse()
    if fault=='time':tr.joint_trajectory.points[1].time_from_start=rospy.Duration(0)
    if fault=='nan':tr.joint_trajectory.points[1].positions[0]=float('nan')
    if fault=='moving_hold':hold.velocities[0]=.01
    if fault=='opening':c=replace(c,opening=.06)
    if fault=='model':c=replace(c,model_sha256='x')
    with pytest.raises(ValueError):validate_lift_trajectory(tr,c,fk,gripper(),hold)


def test_finite_pad_contact_is_allowed_without_exempting_palm():
    fk,c,tr=synthetic(center=[.0004,.0003,.37])
    scene=replace(c.scene,size=np.array([.006,.012,.006]))
    c=replace(c,scene=scene)
    # A stationary closure hold already covered by the same finite pad.
    tr=trajectory([np.zeros(6),np.zeros(6)])
    report=validate_lift_trajectory(tr,c,fk,gripper(),point(np.zeros(6)))
    assert report['allowed_initial_contact_bodies']==['left_finger','right_finger']


def test_obb_outside_finger_does_not_require_whole_target_inside_pad():
    from alicia_flexible_grasp.robot.lift_path_continuity import _intersection_in_pad, _pad_margin
    # A thin rod extends far into the central opening, while only its end
    # intersects the left finger. The whole OBB disk is intentionally large.
    T=np.eye(4); center=np.array([-.025,-.025,-.03]); size=np.array([.092,.002,.004])
    R=np.array([[1.,-1.,0.],[1.,1.,0.],[0.,0.,2**.5]])/2**.5
    assert _pad_margin(T,center,R,size,'left_finger',gripper(),.01) < 0
    assert _intersection_in_pad(T,center,R,size,'left_finger',gripper(),.01)


def test_dropped_t0_outside_joint_limits_is_not_reported_as_validated():
    fk,c,tr=synthetic();tr.joint_trajectory.points[0].positions[0]=3.
    with pytest.raises(LiftPathError,match='joint limits'):
        validate_lift_trajectory(tr,c,fk,gripper(),point(np.zeros(6)))


def test_nonzero_near_parallel_faces_are_unresolved_not_silently_dropped():
    from alicia_flexible_grasp.robot.lift_path_continuity import _intersection_vertices
    angle=2e-13; a=np.array([1.,2.,3.]);a/=np.linalg.norm(a)
    K=np.array([[0.,-a[2],a[1]],[a[2],0.,-a[0]],[-a[1],a[0],0.]])
    R=np.eye(3)+np.sin(angle)*K+(1-np.cos(angle))*(K@K)
    assert _intersection_vertices(np.zeros(3),np.eye(3),np.ones(3),np.zeros(3),R,np.ones(3)) is None


@pytest.mark.parametrize('side',[1.,-1.])
@pytest.mark.parametrize('slack,expected',[(.5e-9,False),(5e-9,False),(20e-9,True)])
def test_pad_penetration_margin_exceeds_vertex_error_enclosure(side,slack,expected):
    from alicia_flexible_grasp.robot.lift_path_continuity import _intersection_in_pad
    T=np.eye(4);size=np.array([.002,.001,.002]);opening=.01
    y=.0003+side*(opening/2+gripper().finger_size_xyz_m[1]/2-size[1]/2-slack)
    assert _intersection_in_pad(T,np.array([.0004,y,-.03]),np.eye(3),size,
                               'left_finger' if side>0 else 'right_finger',
                               gripper(),opening) is expected
