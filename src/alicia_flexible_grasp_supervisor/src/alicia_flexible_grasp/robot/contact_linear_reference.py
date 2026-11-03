"""Configured JTC limits bound to the original contact-linear geometry scope.

The original descent proves nominal continuous CAD and sampled whole-arm
collision. It does not prove a free-space following-error tube or hardware
stopping swept volume. Mapping must preserve these limitations explicitly.
"""
from copy import deepcopy
import math
import re

import rospy
from control_msgs.msg import FollowJointTrajectoryGoal, JointTolerance

from .observation_path_guard import ObservationPathError, wire_digest
from .stationary_following import ARM_NAMES

KIND = 'contact_linear_commanded_geometry_configured_limits_v1'
POLICY = 'contact_linear_existing_configured_limits_no_following_geometry_claim'


def _positive(value, label, zero=False):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value < 0. or (value == 0. and not zero)):
        raise ObservationPathError('CONTACT_LINEAR_REFERENCE_INVALID_'+label)
    return float(value)


def bound_contact_linear_action_goal(plan, audit, constraints, stop_duration, *, reference=None):
    tr = plan.joint_trajectory
    geometry = audit.get('commanded_geometry', {})
    expected = dict(kind=KIND, trajectory_sha256=wire_digest(plan),
        controller_constraints_snapshot=constraints,
        hardware_following_swept_volume_certified=False,
        hardware_stopping_swept_volume_certified=False,
        target_obb_tracking_uncertainty_checked=False,
        joint_limit_uncertainty_checked=False, motion_authorized=False)
    if (any(audit.get(k) != v for k,v in expected.items())
            or audit.get('reference_domain') not in ('wire','physical')
            or audit.get('execution_tracking_contract') != dict(policy=POLICY,
                stop_trajectory_duration_sec=stop_duration)
            or tuple(tr.joint_names) != ARM_NAMES or not tr.points
            or type(audit.get('collision_state_checks')) is not int or audit['collision_state_checks'] <= 0
            or audit.get('collision_trajectory_sha256') != wire_digest(plan)
            or not isinstance(audit.get('gripper_geometry_snapshot'),dict)
            or not audit['gripper_geometry_snapshot']):
        raise ObservationPathError('CONTACT_LINEAR_ACTION_EVIDENCE_MISMATCH')
    if reference is None or tuple(reference.positions) != tuple(audit.get('controller_hold_positions_rad',())):
        raise ObservationPathError('CONTACT_LINEAR_ACTION_REFERENCE_CHANGED')
    if any(tuple(getattr(reference,k,())) != (0.,)*6 for k in ('velocities','accelerations')):
        raise ObservationPathError('CONTACT_LINEAR_ACTION_REFERENCE_NOT_STATIONARY')
    if (geometry.get('trajectory_sha256') != wire_digest(plan)
            or geometry.get('scope') != 'continuous_commanded_Link6_Link7_Link8_only'
            or geometry.get('bridge') != 'stationary_desired_to_first_positive_time_waypoint'
            or tuple(geometry.get('bridge_start_joint_positions_rad',())) != tuple(reference.positions)
            or geometry.get('following_support') is not None
            or not isinstance(geometry.get('plan_id'),str) or not geometry['plan_id']
            or any(re.fullmatch('[0-9a-f]{64}', str(geometry.get(k,''))) is None
                for k in ('scene_sha256','model_sha256'))
            or geometry.get('certified_minimum_clearance_m',-1.) < .003
            or not math.isfinite(float(geometry.get('certified_minimum_clearance_m',float('nan'))))):
        raise ObservationPathError('CONTACT_LINEAR_COMMANDED_GEOMETRY_REQUIRED')
    for key in ('trajectory_sha256','model_sha256','scene_sha256','plan_id','opening_width_m'):
        if audit.get(key) != geometry.get(key):
            raise ObservationPathError('CONTACT_LINEAR_GEOMETRY_BINDING_CHANGED')
    target = geometry.get('target_cad_projection') or {}
    if (target.get('target_envelope_unchanged') is not True
            or target.get('support_and_palm_checks_unchanged') is not True):
        raise ObservationPathError('CONTACT_LINEAR_CAD_MARGIN_CHANGED')
    opening = audit['opening_width_m']
    if isinstance(opening,bool) or not isinstance(opening,(int,float)) or not math.isfinite(opening) or not 0.<=opening<=.05:
        raise ObservationPathError('CONTACT_LINEAR_OPENING_INVALID')
    goal = FollowJointTrajectoryGoal();goal.trajectory=deepcopy(tr)
    for name in ARM_NAMES:
        row = constraints.get(name,{})
        goal.path_tolerance.append(JointTolerance(name=name,position=_positive(row.get('trajectory'),'PATH')))
        goal.goal_tolerance.append(JointTolerance(name=name,position=_positive(row.get('goal'),'GOAL'),
            velocity=_positive(constraints.get('stopped_velocity_tolerance',0.),'VELOCITY',True)))
    goal.goal_time_tolerance=rospy.Duration.from_sec(_positive(constraints.get('goal_time'),'GRACE'))
    _positive(stop_duration,'STOP')
    return goal


def build_contact_linear_audit(plan, geometry, *, reference, constraints, stop_duration,
                               collision_state_checks, gripper_geometry_snapshot,
                               reference_domain='wire'):
    result = {key:deepcopy(geometry.get(key)) for key in
        ('trajectory_sha256','model_sha256','scene_sha256','plan_id','opening_width_m')}
    result.update(kind=KIND, reference_domain=reference_domain,
        commanded_geometry=deepcopy(geometry),
        controller_hold_positions_rad=list(reference.positions),
        controller_constraints_snapshot=deepcopy(constraints),
        execution_tracking_contract=dict(policy=POLICY,stop_trajectory_duration_sec=stop_duration),
        collision_state_checks=collision_state_checks, collision_trajectory_sha256=wire_digest(plan),
        gripper_geometry_snapshot=deepcopy(gripper_geometry_snapshot),
        hardware_following_swept_volume_certified=False,
        hardware_stopping_swept_volume_certified=False,
        target_obb_tracking_uncertainty_checked=False,
        joint_limit_uncertainty_checked=False, motion_authorized=False)
    bound_contact_linear_action_goal(plan,result,constraints,stop_duration,reference=reference)
    return result
