"""Read-only MoveIt lift checks and explicit configured-limit action binding.

This samples complete Noetic curves for whole-arm collision. Continuous
commanded gripper/object/support geometry is a separate pure certificate.
Neither is a proof of hardware following or stopping swept volume.
"""
from copy import deepcopy
import math
import time
import xml.etree.ElementTree as ET

import numpy as np
import rospy
from control_msgs.msg import FollowJointTrajectoryGoal, JointTolerance
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest, GetPlanningScene, GetPlanningSceneRequest
from shape_msgs.msg import SolidPrimitive
from tf.transformations import quaternion_from_matrix

from .observation_path_guard import (
    ObservationPathError, SerialUrdfFk, wire_digest, segment_coefficients, _polynomial, _derivative_bound)
from .stationary_following import ARM_NAMES

KIND = 'closed_lift_commanded_geometry_configured_limits_v1'
POLICY = 'existing_configured_path_goal_limits_no_following_geometry_claim'


def _positive(value,label,zero=False):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value < 0 or (value==0 and not zero):
        raise ObservationPathError('LIFT_INVALID_CONTROLLER_LIMIT: '+label)
    return float(value)


def bound_lift_action_goal(plan,audit,constraints,stop_duration,*,reference=None):
    """Bind existing configured controller tolerances without inventing a proof."""
    domain = audit.get('reference_domain','wire')
    physical = domain == 'physical'
    if (domain not in ('wire','physical') or audit.get('kind') != KIND or audit.get('trajectory_sha256') != wire_digest(plan)
            or audit.get('controller_constraints_snapshot') != constraints
            or audit.get('execution_tracking_contract',{}).get('policy') != POLICY
            or audit['execution_tracking_contract'].get('stop_trajectory_duration_sec') != stop_duration
            or audit.get('stationary_world_target_removed') is not False
            or audit.get('attachment_confirmed') is not False
            or audit.get('whole_arm_collision_method') != 'sampled_exact_Noetic_curve_two_request_local_hypotheses'
            or audit.get('includes_sdk_bridge') is not (not physical)
            or (physical and audit.get('includes_controller_bridge') is not True)
            or audit.get('hardware_following_swept_volume_certified') is not False
            or audit.get('hardware_stopping_swept_volume_certified') is not False
            or tuple(plan.joint_trajectory.joint_names) != ARM_NAMES):
        raise ObservationPathError('LIFT_ACTION_EVIDENCE_MISMATCH')
    hold_key = 'controller_hold_positions_rad' if physical else 'sdk_hold_positions_rad'
    if reference is None or list(reference.positions) != audit.get(hold_key):
        raise ObservationPathError('LIFT_ACTION_SDK_REFERENCE_CHANGED')
    if any(list(getattr(reference,field,())) != [0.]*6 for field in ('velocities','accelerations')):
        raise ObservationPathError('LIFT_ACTION_REFERENCE_NOT_STATIONARY')
    pure = audit.get('pure_lift_geometry',{})
    actual_label = 'physical_with_controller_bridge' if physical else 'shifted_with_sdk_bridge'
    if 'reference_domain' in audit:
        from .lift_path_continuity import ToolBodyBox
        bodies = audit.get('body_geometry_snapshot')
        if not isinstance(bodies,dict):
            raise ObservationPathError('LIFT_MAPPED_BODY_SNAPSHOT_MISSING')
        sources = {}
        for name in ('camera','mount'):
            item = bodies.get(name)
            if item is None:
                continue
            if not isinstance(item,dict) or item.get('frame_id') != 'tool0':
                raise ObservationPathError('LIFT_MAPPED_BODY_FRAME_CHANGED')
            try:
                body = ToolBodyBox(name,item['T_tool_box'],item['size_xyz_m'],item['source_sha256'])
            except (KeyError,TypeError,ValueError) as exc:
                raise ObservationPathError('LIFT_MAPPED_BODY_SNAPSHOT_INVALID') from exc
            sources[name] = body.source_sha256
        missing = sorted({'camera','mount'}-set(sources))
        if (audit.get('missing_rigid_tool_body_models') != missing
                or ('complete_tool_body_coverage' in audit
                    and audit['complete_tool_body_coverage'] is not (not missing))):
            raise ObservationPathError('LIFT_MAPPED_BODY_COVERAGE_CHANGED')
    for label,key in [('nominal','nominal_trajectory_sha256'),(actual_label,'trajectory_sha256')]:
        row=pure.get(label,{})
        if (row.get('trajectory_sha256') != audit.get(key)
                or row.get('scene_sha256') != audit.get('scene_sha256')
                or row.get('model_sha256') != audit.get('model_sha256')
                or row.get('accepted_opening_m') != audit.get('opening_width_m')
                or row.get('stationary_world_target_removed') is not False
                or row.get('attachment_confirmed') is not False):
            raise ObservationPathError('LIFT_ACTION_COMMANDED_GEOMETRY_MISSING')
        if 'reference_domain' in audit and (
                row.get('kind') != 'conditional_continuous_lift_geometry'
                or row.get('bridge') != 'stationary_supplied_hold_to_first_positive_time_point'
                or row.get('plan_id') != audit.get('plan_id')
                or row.get('whole_arm_collision_checked') is not False
                or row.get('hardware_response_verified') is not False
                or row.get('motion_authorized') is not False
                or row.get('grasp_success') is not False
                or row.get('missing_rigid_tool_body_models') != missing
                or row.get('complete_tool_body_coverage') is not (not missing)
                or row.get('body_sources',{}) != sources):
            raise ObservationPathError('LIFT_MAPPED_GEOMETRY_SCOPE_CHANGED')
        if ('reference_domain' in audit and label == actual_label
                and row.get('bridge_start_rad') != list(reference.positions)):
            raise ObservationPathError('LIFT_MAPPED_BRIDGE_CHANGED')
    goal=FollowJointTrajectoryGoal();goal.trajectory=deepcopy(plan.joint_trajectory)
    for name in ARM_NAMES:
        row=constraints.get(name,{})
        goal.path_tolerance.append(JointTolerance(name=name,position=_positive(row.get('trajectory'),name+' path')))
        goal.goal_tolerance.append(JointTolerance(name=name,position=_positive(row.get('goal'),name+' goal'),
            velocity=_positive(constraints.get('stopped_velocity_tolerance',0.),'stopped velocity',True)))
    goal.goal_time_tolerance=rospy.Duration.from_sec(_positive(constraints.get('goal_time'),'goal time'))
    _positive(stop_duration,'stop duration')
    return goal


def _pose(T):
    p=Pose();p.position.x,p.position.y,p.position.z=map(float,T[:3,3])
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,quaternion_from_matrix(T))
    return p


class ClosedLiftMoveItAdapter:
    """Current-scene, request-local target hypotheses; never edits global scene."""
    def __init__(self,model,check_state,read_scene,*,clock=time.monotonic,max_seconds=15.,max_requests=8192):
        self.fk=SerialUrdfFk(model,ARM_NAMES)
        links={l.get('name'):l for l in ET.fromstring(model).findall('link')}
        if any(n not in links for n in ('base_link','tool0','Link7','Link8')) or links['tool0'].find('collision') is not None:
            raise ObservationPathError('LIFT_REQUEST_LOCAL_TOOL0_FRAME_CONTRACT')
        self.check_state,self.read_scene=check_state,read_scene
        self.clock,self.max_seconds,self.max_requests=clock,max_seconds,max_requests

    @classmethod
    def connect(cls,model):
        # Both are read-only MoveIt APIs; no planning-scene publisher/apply RPC.
        rospy.wait_for_service('/check_state_validity',timeout=2.)
        rospy.wait_for_service('/get_planning_scene',timeout=2.)
        check=rospy.ServiceProxy('/check_state_validity',GetStateValidity)
        scene=rospy.ServiceProxy('/get_planning_scene',GetPlanningScene)
        return cls(model,check,scene)

    def scene_digest(self):
        req=GetPlanningSceneRequest()
        req.components.components=(PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            |PlanningSceneComponents.OCTOMAP|PlanningSceneComponents.TRANSFORMS
            |PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            |PlanningSceneComponents.LINK_PADDING_AND_SCALING
            |PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        response=self.read_scene(req)
        # Request excludes ordinary live joint state; preserve every returned
        # existing attached object and every environmental collision field.
        snapshot=deepcopy(response.scene)
        snapshot.robot_state.joint_state=type(snapshot.robot_state.joint_state)()
        snapshot.robot_state.multi_dof_joint_state=type(snapshot.robot_state.multi_dof_joint_state)()
        return wire_digest(snapshot)

    def _request(self,q,start,frozen,hypothesis):
        req=GetStateValidityRequest();req.robot_state=deepcopy(start)
        state=req.robot_state;state.is_diff=True
        names=list(state.joint_state.name)
        positions=np.asarray(state.joint_state.position,dtype=float)
        q=np.asarray(q,dtype=float)
        if (len(set(names)) != len(names) or positions.shape != (len(names),)
                or not np.isfinite(positions).all() or q.shape != (6,) or not np.isfinite(q).all()
                or state.multi_dof_joint_state.joint_names or state.multi_dof_joint_state.transforms
                or not set(ARM_NAMES+('right_finger',)).issubset(names)):
            raise ObservationPathError('LIFT_MOVEIT_START_STATE_INCOMPLETE')
        mapping=dict(zip(ARM_NAMES,q));mapping.update(right_finger=frozen.opening,left_finger=-frozen.opening)
        state.joint_state.position=[float(mapping.get(n,v)) for n,v in zip(names,state.joint_state.position)]
        attached=AttachedCollisionObject();attached.link_name='tool0'
        # tool0 has no collision geometry. Attaching a world hypothesis to
        # base_link would implicitly exempt its real base collision, so keep
        # a request-local body at a base_link POSE using this empty frame.
        attached.object.header.frame_id='base_link' if hypothesis=='world' else 'tool0'
        attached.object.id='lift_diagnostic_'+hypothesis+'_'+frozen.scene.digest[:16]
        if any(a.object.id==attached.object.id for a in state.attached_collision_objects):
            raise ObservationPathError('LIFT_REQUEST_LOCAL_OBJECT_ID_COLLISION')
        T=np.eye(4)
        if hypothesis=='world':T[:3,:3]=frozen.scene.rotation;T[:3,3]=frozen.scene.center
        else:T=np.array(frozen.T_tool_object,copy=True)
        attached.object.pose=_pose(T)
        primitive=SolidPrimitive(type=SolidPrimitive.BOX,dimensions=frozen.scene.size.tolist())
        origin=Pose();origin.orientation.w=1.
        attached.object.primitives=[primitive];attached.object.primitive_poses=[origin]
        attached.object.operation=CollisionObject.ADD
        # This coarse MoveIt permission is always paired with the continuous
        # finite-pad intersection proof; it is not a whole-finger exemption.
        attached.touch_links=['Link7','Link8']
        state.attached_collision_objects.append(attached)
        req.group_name=''  # entire robot, including base and both fingers
        return req

    @staticmethod
    def _samples(path,hold):
        points=list(path.joint_trajectory.points)
        first=next(i for i,p in enumerate(points) if p.time_from_start.to_sec()>0)
        if first:yield np.asarray(points[0].positions),'dropped_t0'
        pairs=[(hold,points[first],points[first].time_from_start.to_sec(),'bridge')]
        pairs.extend((a,b,b.time_from_start.to_sec()-a.time_from_start.to_sec(),str(i))
                     for i,(a,b) in enumerate(zip(points[first:],points[first+1:]),first))
        for a,b,duration,label in pairs:
            coeff=segment_coefficients(a,b,duration,6)
            count=max(1,int(math.ceil(float(np.max(_derivative_bound(coeff,0.,1.)))/.002)))
            if count>4096:raise ObservationPathError('LIFT_MOVEIT_SPLINE_SAMPLE_BUDGET')
            for t in np.linspace(0.,1.,count+1):yield _polynomial(coeff,float(t)),label

    def __call__(self,*,nominal,shifted,frozen,start_state,sdk_hold,bodies,gripper,constraints,stop_duration,
                 reference_domain='wire'):
        if reference_domain not in ('wire','physical'):
            raise ObservationPathError('LIFT_REFERENCE_DOMAIN_INVALID')
        started=self.clock();before=self.scene_digest();requests=0
        if self.fk.model_sha256 != frozen.model_sha256:
            raise ObservationPathError('LIFT_MOVEIT_MODEL_CHANGED')
        nominal_hold=deepcopy(sdk_hold);nominal_hold.positions=frozen.measured.tolist()
        for path,label,hold in ((nominal,'nominal',nominal_hold),(shifted,'shifted',sdk_hold)):
            for q,segment in self._samples(path,hold):
                # SerialUrdfFk also rejects limits, nonfinite and wrong length.
                self.fk(q)
                for hypothesis in ('world','hypothetical_carried'):
                    if requests>=self.max_requests or self.clock()-started>self.max_seconds:
                        raise ObservationPathError('LIFT_MOVEIT_READ_ONLY_CHECK_BUDGET')
                    result=self.check_state(self._request(q,start_state,frozen,hypothesis))
                    requests+=1
                    if result.valid is not True:
                        raise ObservationPathError('LIFT_MOVEIT_COLLISION: '+label+'/'+segment+'/'+hypothesis)
        after=self.scene_digest()
        if before!=after:raise ObservationPathError('LIFT_MOVEIT_SCENE_CHANGED_DURING_CHECK')
        report = dict(kind=KIND,trajectory_sha256=wire_digest(shifted),
            nominal_trajectory_sha256=wire_digest(nominal),scene_sha256=frozen.scene.digest,
            model_sha256=frozen.model_sha256,epoch_ns=frozen.epoch_ns,opening_width_m=frozen.opening,
            whole_arm_collision_method='sampled_exact_Noetic_curve_two_request_local_hypotheses',
            whole_arm_state_checks=requests,maximum_sampled_joint_step_rad=.002,
            includes_sdk_bridge=True,stationary_world_target_removed=False,attachment_confirmed=False,
            sdk_hold_positions_rad=list(sdk_hold.positions),controller_constraints_snapshot=deepcopy(constraints),
            execution_tracking_contract=dict(policy=POLICY,stop_trajectory_duration_sec=stop_duration),
            moveit_scene_sha256=before,
            hardware_following_swept_volume_certified=False,hardware_stopping_swept_volume_certified=False,
            missing_rigid_tool_body_models=sorted({'camera','mount'}-{b.name for b in bodies}),
            additional_rigid_tool_bodies_whole_arm_checked=False,
            motion_authorized=False,elapsed_seconds=self.clock()-started)
        if reference_domain == 'physical':
            report.update(reference_domain='physical',includes_sdk_bridge=False,
                includes_controller_bridge=True,controller_hold_positions_rad=list(sdk_hold.positions))
            report.pop('sdk_hold_positions_rad')
        return report
