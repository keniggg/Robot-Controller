"""Explicit observation action contracts; never enable/disable robot torque.

MoveIt remains responsible for planning and collision validation. This
adapter submits only the final proof-bound joint trajectory to the same
trajectory controller, with the exact per-goal path tolerances used by the
geometry proof. It does not implement a planning or `go()` fallback.
"""
import math
import time
from copy import deepcopy

import numpy as np
import rospy
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryGoal, JointTolerance
from trajectory_msgs.msg import JointTrajectory

from .observation_path_guard import ObservationPathError, wire_digest
from .observation_tracking_contract import (
    tracking_contract_candidates, validate_command_derivatives, TRAJECTORY_STOP_POLICY,
)
from .contact_tracking_contract import (
    POLICY as CONTACT_POLICY, validate_contact_tracking_contract_binding,
)


def bound_action_goal(plan, audit, constraints, stop_duration, *, reference=None):
    """Reject altered/missing proof or mismatched execution tolerances."""
    names = list(plan.joint_trajectory.joint_names)
    if len(names) != 6 or set(names) != {'Joint%d' % i for i in range(1, 7)}:
        raise ObservationPathError('observation action requires exactly six arm joints')
    if audit.get('trajectory_sha256') != wire_digest(plan):
        raise ObservationPathError('observation action/proof trajectory mismatch')
    contract = audit.get('execution_tracking_contract')
    bounds = None
    if isinstance(contract, dict) and contract.get('policy') == TRAJECTORY_STOP_POLICY:
        if reference is None:
            raise ObservationPathError('trajectory stopping proof needs the bound controller reference')
        bounds = validate_command_derivatives(plan, reference)
        if audit.get('command_derivative_bounds') != bounds:
            raise ObservationPathError('trajectory stopping derivative proof changed')
    is_contact = isinstance(contract, dict) and contract.get('policy') == CONTACT_POLICY
    if is_contact:
        if reference is None:
            raise ObservationPathError('contact action needs the bound controller reference')
        validate_contact_tracking_contract_binding(contract, plan, reference, constraints, stop_duration)
        if (audit.get('compensation_pose_index') != 2
                or audit.get('command_derivative_bounds') != contract['command_derivative_bounds']
                or audit.get('model_sha256') != contract['model_sha256']):
            raise ObservationPathError('contact action scope or model changed')
    else:
        choices = tracking_contract_candidates(constraints, names, stop_duration, command_bounds=bounds)
        if not isinstance(contract, dict) or contract not in choices:
            raise ObservationPathError('observation action has no exact tracking contract')
    following = audit.get('following_support') or {}
    errors = np.asarray(following.get('joint_error_bounds_rad', []), dtype=float)
    if (errors.shape != (6,)
            or not np.array_equal(errors, np.asarray(contract['joint_error_bounds_rad']))
            or following.get('includes_controller_bridge') is not True
            or not math.isfinite(float(following.get('minimum_support_clearance_lower_bound_m', float('nan'))))
            or float(following['minimum_support_clearance_lower_bound_m']) < .003
            or not math.isfinite(float(audit.get('certified_minimum_clearance_m', float('nan'))))
            or float(audit['certified_minimum_clearance_m']) < .003):
        raise ObservationPathError('observation action lacks complete conditional CAD proof')
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = deepcopy(plan.joint_trajectory)
    for name, value in zip(names, contract['path_position_tolerance_rad']):
        goal.path_tolerance.append(JointTolerance(name=name, position=value))
        row = constraints[name]
        endpoint = row.get('goal')
        if (isinstance(endpoint, bool) or not isinstance(endpoint, (int, float))
                or not math.isfinite(endpoint) or endpoint <= 0.):
            raise ObservationPathError('missing positive endpoint tolerance for '+name)
        if is_contact:
            endpoint = contract['goal_position_tolerance_rad'][names.index(name)]
        velocity = constraints.get('stopped_velocity_tolerance', 0.)
        if (isinstance(velocity, bool) or not isinstance(velocity, (int, float))
                or not math.isfinite(velocity) or velocity < 0.):
            raise ObservationPathError('invalid stopped velocity tolerance')
        goal.goal_tolerance.append(JointTolerance(name=name, position=endpoint, velocity=velocity))
    grace = constraints.get('goal_time')
    if (isinstance(grace, bool) or not isinstance(grace, (int, float))
            or not math.isfinite(grace) or not 0. < grace <= 30.):
        raise ObservationPathError('invalid observation goal time tolerance')
    goal.goal_time_tolerance = rospy.Duration.from_sec(grace)
    return goal


class ContractedTrajectoryExecutor:
    """Injected action/hold transports make failure ordering testable offline.

    Noetic path/goal tolerance aborts clear the active action without replacing
    its trajectory. Cancelling that finished goal has no effect. Therefore an
    ABORTED or timed-out observation explicitly installs the controller's
    ordinary stopping hold, preserving torque. Manual takeover/external
    preemption receives no hold publication from this adapter.
    """
    def __init__(self, client, hold_publisher, *, clock=time.monotonic):
        self.client, self.hold_publisher, self.clock = client, hold_publisher, clock
        self.active = False

    @classmethod
    def connect(cls):
        import actionlib
        from control_msgs.msg import FollowJointTrajectoryAction
        client = actionlib.SimpleActionClient('/alicia_controller/follow_joint_trajectory',
                                              FollowJointTrajectoryAction)
        if not client.wait_for_server(rospy.Duration.from_sec(2.)):
            raise ObservationPathError('observation action controller is unavailable')
        publisher = rospy.Publisher('/alicia_controller/command', JointTrajectory,
                                    queue_size=1, latch=False)
        deadline = time.monotonic()+2.
        while publisher.get_num_connections() < 1 and time.monotonic() < deadline:
            if rospy.is_shutdown():
                raise ObservationPathError('shutdown while connecting controller hold')
            rospy.sleep(.02)
        if publisher.get_num_connections() < 1:
            raise ObservationPathError('observation stopping-hold transport unavailable')
        return cls(client, publisher)

    def cancel(self):
        if self.active:
            self.client.cancel_goal()

    def execute(self, goal, *, before_send, still_authorized, execution_monitor=None,
                mapped_submission=None):
        if not callable(before_send) or not callable(still_authorized):
            raise ObservationPathError('observation action requires live authority callbacks')
        if self.hold_publisher.get_num_connections() < 1:
            raise ObservationPathError('observation stopping-hold transport disconnected')
        if not still_authorized():
            return False, 'observation authority revoked before submission'
        duration = goal.trajectory.points[-1].time_from_start.to_sec()
        lead = 0.
        if mapped_submission is not None:
            lead = max(0., goal.trajectory.header.stamp.to_sec()-float(rospy.get_time()))
        deadline = self.clock()+lead+duration+goal.goal_time_tolerance.to_sec()+2.
        if mapped_submission is not None:
            deadline = min(deadline, mapped_submission.lease.local_deadline)
        sent = False
        manager, original_send = None, None
        def cancel_this_goal():
            if mapped_submission is None:
                self.client.cancel_goal()
                return
            envelope = mapped_submission.lease.armed_envelope
            if envelope is None:
                return
            # init_goal calls the publisher before returning its handle. If
            # publication raises after transmission, SimpleActionClient.gh
            # may still name the preceding goal. Cancel only this exact ID.
            from actionlib_msgs.msg import GoalID
            self.client.action_client.pub_cancel.publish(
                GoalID(id=envelope.goal_id.id, stamp=rospy.Time(0)))
        try:
            # Revalidate the final controller reference after transport setup,
            # immediately before this exact action is sent.
            before_send()
            if execution_monitor is not None:
                execution_monitor()
            if not still_authorized():
                return False, 'observation authority revoked before submission'
            self.active = True
            # A transport can raise after publishing. Treat an attempted
            # send as possibly accepted, so cleanup cannot miss that case.
            if mapped_submission is not None:
                from .physical_reference_client import MappedTrajectorySubmission
                from .physical_reference_contract import message_bytes
                if type(mapped_submission) is not MappedTrajectorySubmission:
                    raise ObservationPathError('PHYSICAL_TYPED_SUBMISSION_REQUIRED')
                manager = getattr(getattr(self.client, 'action_client', None), 'manager', None)
                original_send = getattr(manager, 'send_goal_fn', None)
                if (manager is None or not callable(original_send)
                        or not callable(getattr(manager, 'register_send_goal_fn', None))
                        or not callable(getattr(getattr(self.client.action_client,
                            'pub_cancel', None), 'publish', None))):
                    raise ObservationPathError('PHYSICAL_EXACT_ACTION_ENVELOPE_HOOK_UNAVAILABLE')
                expected_goal = message_bytes(goal)
                calls = [0]
                def armed_send(envelope):
                    nonlocal sent
                    calls[0] += 1
                    if calls[0] != 1 or message_bytes(envelope.goal) != expected_goal:
                        raise ObservationPathError('PHYSICAL_ACTION_ENVELOPE_CHANGED_OR_REPLAYED')
                    if not still_authorized():
                        raise ObservationPathError('PHYSICAL_ACTION_AUTHORITY_REVOKED')
                    mapped_submission.arm(envelope)
                    if not still_authorized():
                        raise ObservationPathError('PHYSICAL_ACTION_AUTHORITY_REVOKED_AFTER_ARM')
                    # A publish exception may occur after transmission.
                    sent = True
                    original_send(envelope)
                manager.register_send_goal_fn(armed_send)
            else:
                sent = True
            self.client.send_goal(goal)
            if manager is not None:
                manager.register_send_goal_fn(original_send)
                manager = None
                if calls[0] != 1:
                    raise ObservationPathError('PHYSICAL_ACTION_ENVELOPE_NOT_OBSERVED')
            while self.clock() < deadline:
                if not still_authorized() or rospy.is_shutdown():
                    cancel_this_goal()
                    return False, 'observation authority revoked during execution'
                if execution_monitor is not None:
                    execution_monitor()
                if mapped_submission is not None:
                    mapped_submission.check_active()
                if self.client.wait_for_result(rospy.Duration.from_sec(.05)):
                    if execution_monitor is not None:
                        execution_monitor()
                    result, state = self.client.get_result(), self.client.get_state()
                    if mapped_submission is not None:
                        mapped_submission.confirm_result(state, result)
                    if (state == GoalStatus.SUCCEEDED and result is not None
                            and result.error_code == result.SUCCESSFUL):
                        return True, 'observation action succeeded with bound path tolerances'
                    if state not in (GoalStatus.PREEMPTED, GoalStatus.RECALLED, GoalStatus.REJECTED):
                        if still_authorized():
                            self.hold_publisher.publish(JointTrajectory())
                    return False, 'observation action failed state=%s code=%s reason=%s' % (
                        state, getattr(result, 'error_code', None), getattr(result, 'error_string', ''))
            cancel_this_goal()
            if still_authorized():
                self.hold_publisher.publish(JointTrajectory())
            return False, 'observation action deadline exceeded; controller hold requested'
        except Exception:
            if sent:
                cancel_this_goal()
                if still_authorized():
                    self.hold_publisher.publish(JointTrajectory())
            raise
        finally:
            if manager is not None:
                manager.register_send_goal_fn(original_send)
            self.active = False
