import sys
import math
import inspect
import hashlib
import json
import time
from collections import OrderedDict
from copy import deepcopy
from types import SimpleNamespace

import rospy
from alicia_flexible_grasp.robot.actuation_bootstrap import bounded_observation_prefix

class MoveItPlanner:
    OBSERVATION_BRANCH_HINT_MAX_AGE_SEC = 120.0
    OBSERVATION_BRANCH_HINT_CAPACITY = 128
    DEFAULT_CANDIDATE_ORIENTATIONS_XYZW = (
        (0.0, 0.0, 0.0, 1.0),
        (0.0, 0.7071, 0.0, 0.7071),
        (0.0, -0.7071, 0.0, 0.7071),
        (0.7071, 0.0, 0.0, 0.7071),
        (-0.7071, 0.0, 0.0, 0.7071),
        (0.0, 0.0, 0.7071, 0.7071),
        (0.0, 0.0, -0.7071, 0.7071),
    )

    def __init__(self, manipulator_group='alicia', gripper_group='hand', velocity=0.3):
        self.ready = False
        self.error = None
        self.manipulator_group = str(manipulator_group)
        self.pose_fallback_enabled = self._bool_param('~pose_fallback_enabled', '/robot/pose_fallback_enabled', True)
        self.position_only_fallback_enabled = self._bool_param('~position_only_fallback_enabled', '/robot/position_only_fallback_enabled', True)
        self.position_only_execute_enabled = self._bool_param('~position_only_execute_enabled', '/robot/position_only_execute_enabled', False)
        self.orientation_fallback_enabled = self._bool_param('~orientation_fallback_enabled', '/robot/orientation_fallback_enabled', True)
        self.cached_plan_position_tolerance_m = self._float_param(
            '~cached_plan_position_tolerance_m',
            '/robot/cached_plan_position_tolerance_m',
            0.002,
        )
        self.cached_plan_orientation_tolerance_rad = self._float_param(
            '~cached_plan_orientation_tolerance_rad',
            '/robot/cached_plan_orientation_tolerance_rad',
            0.02,
        )
        self.execution_goal_tolerance_rad = self._float_param(
            '~execution_goal_tolerance_rad',
            '/robot/execution_goal_tolerance_rad',
            0.03,
        )
        self.execution_goal_tolerance_slack_rad = self._float_param(
            '~execution_goal_tolerance_slack_rad',
            '/robot/execution_goal_tolerance_slack_rad',
            0.005,
        )
        self.strict_execution_retime_enabled = self._bool_param(
            '~strict_execution_retime_enabled',
            '/robot/strict_execution_retime_enabled',
            True,
        )
        self.strict_execution_velocity_scaling = self._float_param(
            '~strict_execution_velocity_scaling',
            '/robot/strict_execution_velocity_scaling',
            0.20,
        )
        self.strict_execution_acceleration_scaling = self._float_param(
            '~strict_execution_acceleration_scaling',
            '/robot/strict_execution_acceleration_scaling',
            0.30,
        )
        self.strict_execution_max_joint_velocity_rad_s = self._float_param(
            '~strict_execution_max_joint_velocity_rad_s',
            '/robot/strict_execution_max_joint_velocity_rad_s',
            0.08,
        )
        self.strict_execution_joint_velocity_limits_rad_s = (
            self._joint_velocity_limits_param(
                '~strict_execution_joint_velocity_limits_rad_s',
                '/robot/strict_execution_joint_velocity_limits_rad_s',
                {},
            )
        )
        self.strict_execution_min_duration_sec = self._float_param(
            '~strict_execution_min_duration_sec',
            '/robot/strict_execution_min_duration_sec',
            3.0,
        )
        self.strict_execution_start_tolerance_rad = self._float_param(
            '~strict_execution_start_tolerance_rad',
            '/robot/strict_execution_start_tolerance_rad',
            0.08,
        )
        self.cartesian_eef_step_m = self._float_param(
            '~cartesian_eef_step_m', '/robot/cartesian_eef_step_m', 0.003
        )
        self.cartesian_jump_threshold = self._float_param(
            '~cartesian_jump_threshold', '/robot/cartesian_jump_threshold', 0.0
        )
        self.cartesian_min_fraction = self._float_param(
            '~cartesian_min_fraction', '/robot/cartesian_min_fraction', 0.98
        )
        self.cartesian_max_segment_m = self._float_param(
            '~cartesian_max_segment_m', '/robot/cartesian_max_segment_m', 0.08
        )
        self.cartesian_velocity_scaling = self._float_param(
            '~cartesian_velocity_scaling', '/robot/cartesian_velocity_scaling', 0.20
        )
        self.cartesian_acceleration_scaling = self._float_param(
            '~cartesian_acceleration_scaling', '/robot/cartesian_acceleration_scaling', 0.30
        )
        self.candidate_orientations = self._orientation_param(
            '~pregrasp_candidate_orientations_xyzw',
            '/robot/pregrasp_candidate_orientations_xyzw',
            self.DEFAULT_CANDIDATE_ORIENTATIONS_XYZW,
        )
        self._last_pose_plan = None
        # Enabled only on the gateway's separate observation planner. These
        # are joint-goal hints, never permission to replay a checked trajectory.
        self.observation_branch_hints_enabled = False
        self.observation_path_guard_required = False
        self.observation_path_validator = None
        # The gateway supplies fresh, transmitted stationary controller state.
        # It is a bridge reference only, never a replacement planning start.
        self.controller_reference_guard_required = False
        self.controller_reference_provider = None
        self.controller_reference_completion_callback = None
        self._observation_branch_hints = OrderedDict()
        try:
            import moveit_commander
            self.moveit_commander = moveit_commander
            moveit_commander.roscpp_initialize(sys.argv)
            self.robot = moveit_commander.RobotCommander()
            self.scene = moveit_commander.PlanningSceneInterface()
            self.manipulator = moveit_commander.MoveGroupCommander(manipulator_group)
            self.gripper = moveit_commander.MoveGroupCommander(gripper_group)
            self.manipulator.set_max_velocity_scaling_factor(float(velocity))
            self.manipulator.set_max_acceleration_scaling_factor(0.5)
            planning_time = float(rospy.get_param('~planning_time', rospy.get_param('/robot/planning_time', 2.0)))
            self.orientation_resolution_planning_time = max(
                0.05,
                float(
                    rospy.get_param(
                        '~orientation_resolution_planning_time',
                        rospy.get_param(
                            '/robot/orientation_resolution_planning_time',
                            planning_time,
                        ),
                    )
                ),
            )
            self.orientation_resolution_step_rad = max(
                1e-3,
                float(
                    rospy.get_param(
                        '~orientation_resolution_step_rad',
                        rospy.get_param(
                            '/robot/orientation_resolution_step_rad',
                            self.cached_plan_orientation_tolerance_rad,
                        ),
                    )
                ),
            )
            self.orientation_resolution_max_candidates = max(
                2,
                min(
                    256,
                    int(
                        rospy.get_param(
                            '~orientation_resolution_max_candidates',
                            rospy.get_param(
                                '/robot/orientation_resolution_max_candidates',
                                96,
                            ),
                        )
                    ),
                ),
            )
            self.orientation_resolution_ik_timeout_sec = max(
                1e-3,
                float(
                    rospy.get_param(
                        '~orientation_resolution_ik_timeout_sec',
                        rospy.get_param(
                            '/robot/orientation_resolution_ik_timeout_sec',
                            0.05,
                        ),
                    )
                ),
            )
            self.orientation_resolution_repeatability_tolerance_rad = max(
                0.0,
                float(
                    rospy.get_param(
                        '~orientation_resolution_repeatability_tolerance_rad',
                        rospy.get_param(
                            (
                                '/robot/orientation_resolution_'
                                'repeatability_tolerance_rad'
                            ),
                            1e-6,
                        ),
                    )
                ),
            )
            self.strict_pose_planning_time = max(
                0.05,
                float(
                    rospy.get_param(
                        '~strict_pose_planning_time',
                        rospy.get_param('/robot/strict_pose_planning_time', 0.25),
                    )
                ),
            )
            planning_attempts = int(rospy.get_param('~planning_attempts', rospy.get_param('/robot/planning_attempts', 1)))
            self.manipulator.set_planning_time(max(0.5, planning_time))
            self.manipulator.set_num_planning_attempts(max(1, planning_attempts))
            self.ready = True
        except Exception as exc:
            self.error = str(exc)
            rospy.logwarn('MoveItPlanner not ready: %s', self.error)

    def move_to_pose(self, pose_stamped_or_pose, execute=True, allow_fallbacks=True):
        if not self.ready:
            return False, self.error or 'MoveIt not ready'
        pose = getattr(pose_stamped_or_pose, 'pose', pose_stamped_or_pose)
        target_text = self._pose_xyz_text(pose)
        action = 'execute' if execute else 'plan'
        action_done = 'executed' if execute else 'planned'
        failed_attempts = []
        try:
            if execute:
                cached_ok, cached_message = self._execute_cached_pose_plan(pose, target_text)
                if cached_message:
                    return cached_ok, cached_message
                plan_ok, plan_message = self.move_to_pose(
                    pose,
                    execute=False,
                    allow_fallbacks=allow_fallbacks,
                )
                if not plan_ok:
                    return False, 'execute planning failed before motion: %s' % plan_message
                cached_ok, cached_message = self._execute_cached_pose_plan(pose, target_text)
                if cached_message:
                    return cached_ok, cached_message
                return False, 'execute failed: no executable cached plan after planning; %s' % target_text

            strict_planning_time = None
            if not execute and not allow_fallbacks:
                strict_planning_time = float(getattr(self, 'strict_pose_planning_time', 0.25))
            if self._attempt_pose_target(pose, execute, planning_time=strict_planning_time):
                return True, '%s: %s%s' % (action_done, target_text, self._cached_plan_metrics_text())
            failed_attempts.append('strict pose')

            if (
                allow_fallbacks
                and self._pose_fallbacks_enabled()
                and getattr(self, 'orientation_fallback_enabled', True)
            ):
                labels = []
                for label, orientation in self._candidate_orientations(pose):
                    labels.append(label)
                    candidate = self._pose_with_orientation(pose, orientation)
                    if self._attempt_pose_target(
                        candidate,
                        execute,
                        plan_kind='candidate orientation %s' % label,
                    ):
                        return True, '%s with candidate orientation %s: %s%s' % (
                            action_done,
                            label,
                            target_text,
                            self._cached_plan_metrics_text(),
                        )
                if labels:
                    failed_attempts.append('candidate orientations %s' % ','.join(labels))

            if allow_fallbacks and self._position_only_fallback_allowed(execute):
                if self._attempt_position_target(pose, execute):
                    return True, '%s with position-only fallback: %s%s' % (
                        action_done,
                        target_text,
                        self._cached_plan_metrics_text(),
                    )
                failed_attempts.append('position-only')
            elif allow_fallbacks and execute and getattr(self, 'position_only_fallback_enabled', True):
                failed_attempts.append('position-only disabled for execute')

            attempts_text = ', '.join(failed_attempts) if failed_attempts else 'strict pose'
            return False, (
                '%s failed: %s; target unreachable or pose orientation invalid after %s; '
                'check workspace, candidate orientation, handeye/base transform, collision scene, and current joint state'
            ) % (action, target_text, attempts_text)
        except Exception as exc:
            return False, 'move_to_pose exception: %s; %s' % (exc, target_text)
        finally:
            self._clear_targets()

    def move_to_pose_linear(self, pose_stamped_or_pose, execute=True):
        if not self.ready:
            return False, self.error or 'MoveIt not ready'
        pose = getattr(pose_stamped_or_pose, 'pose', pose_stamped_or_pose)
        target_text = self._pose_xyz_text(pose)
        try:
            if execute:
                cached_ok, cached_message = self._execute_cached_pose_plan(
                    pose,
                    target_text,
                    required_kind='cartesian',
                )
                if cached_message:
                    return cached_ok, cached_message
                return False, 'linear execute failed: no matching Cartesian plan; %s' % target_text

            # A new planning request supersedes every previously cached pose
            # trajectory even when validation or planning below fails.
            self._last_pose_plan = None
            current = self.manipulator.get_current_pose()
            current_pose = getattr(current, 'pose', current)
            distance = self._pose_position_distance(current_pose, pose)
            max_segment = max(0.0, float(getattr(self, 'cartesian_max_segment_m', 0.08)))
            if max_segment > 0.0 and distance > max_segment:
                return False, (
                    'Cartesian segment %.3fm exceeds limit %.3fm; %s'
                    % (distance, max_segment, target_text)
                )

            plan, fraction = self._compute_cartesian_plan(pose)
            min_fraction = min(1.0, max(0.0, float(getattr(self, 'cartesian_min_fraction', 0.98))))
            if plan is None or not self._plan_success(plan) or fraction < min_fraction:
                self._last_pose_plan = None
                return False, (
                    'Cartesian path incomplete fraction=%.3f < %.3f; %s'
                    % (fraction, min_fraction, target_text)
                )
            plan = self._retime_cartesian_plan(plan)
            self._remember_pose_plan(pose, plan, 'cartesian')
            return True, 'planned Cartesian line fraction=%.3f distance=%.3fm: %s%s' % (
                fraction,
                distance,
                target_text,
                self._cached_plan_metrics_text(),
            )
        except Exception as exc:
            self._last_pose_plan = None
            return False, 'Cartesian move exception: %s; %s' % (exc, target_text)

    def execute_cached_strict_pose(self, pose_stamped_or_pose):
        """Execute only the matching trajectory produced by strict pose planning.

        This is deliberately separate from ``move_to_pose(execute=True)``.  It
        never plans, calls ``go()``, or tries any fallback.  Callers must first
        create the cache with ``move_to_pose(..., execute=False,
        allow_fallbacks=False)``.
        """
        if not self.ready:
            return False, self.error or 'MoveIt not ready'
        pose = getattr(pose_stamped_or_pose, 'pose', pose_stamped_or_pose)
        target_text = self._pose_xyz_text(pose)
        try:
            _cached, mismatch = self._matching_cached_strict_pose_plan(pose)
            if mismatch:
                return False, 'strict cached execute blocked: %s; %s' % (
                    mismatch,
                    target_text,
                )
            ok, message = self._execute_cached_pose_plan(
                pose,
                target_text,
                required_kind='strict pose',
            )
            if message:
                return ok, message
            # The strict matcher above succeeded, so reaching this branch can
            # only mean the cache changed before it could be claimed.
            return False, (
                'strict cached execute blocked: cached plan changed before execution; %s'
                % target_text
            )
        except Exception as exc:
            return False, 'strict cached execute exception: %s; %s' % (exc, target_text)
        finally:
            self._clear_targets()

    def execute_cached_observation_prefix(self, pose_stamped_or_pose):
        """Consume a matching strict plan, executing at most 0.025 rad per joint."""
        pose = getattr(pose_stamped_or_pose, 'pose', pose_stamped_or_pose)
        cached, mismatch = self._matching_cached_strict_pose_plan(pose)
        if mismatch:
            return False, 'ACTUATION_PREFIX_PLAN_INVALID: ' + mismatch
        # Never retain the full observation trajectory after this partial move.
        self._last_pose_plan = None
        try:
            prefix = bounded_observation_prefix(cached['plan'])
            current = list(self.manipulator.get_current_joint_values())
            origin = prefix.joint_trajectory.points[0].positions
            if len(current) != 6 or any(not math.isfinite(v) for v in current):
                return False, 'ACTUATION_PREFIX_FEEDBACK_INVALID'
            if max(abs(a-b) for a, b in zip(origin, current)) > 0.003:
                return False, 'ACTUATION_PREFIX_START_CHANGED: planned=%s measured=%s; replan from measured feedback' % (list(origin), current)
            if not getattr(self, 'strict_execution_retime_enabled', False):
                return False, 'ACTUATION_PREFIX_RETIMING_REQUIRED'
            prefix, reason = self._retime_strict_execution_plan(prefix)
            if prefix is None:
                return False, 'ACTUATION_PREFIX_RETIMING_FAILED: ' + reason
            prefix, reference_evidence, reference_error = (
                self._prepare_controller_reference_timing(prefix))
            if reference_error:
                return False, reference_error
            guard_error = self._observation_path_guard_error(prefix)
            if guard_error:
                return False, 'ACTUATION_PREFIX_PATH_BLOCKED: ' + guard_error
            reference_error = self._controller_reference_execution_error(
                prefix, reference_evidence)
            if reference_error:
                return False, reference_error
            # execute() waits for this bounded trajectory only. No full-plan
            # continuation, go() fallback, stop(), or torque command is issued.
            if getattr(self, 'observation_tracking_contract_required', False):
                ok, contract_message = self._execute_observation_contract(prefix, reference_evidence)
                if not ok:
                    return False, contract_message
            else:
                ok = bool(self.manipulator.execute(prefix, wait=True))
            if ok:
                completion_error = self._controller_reference_completion_error(prefix)
                if completion_error:
                    return False, completion_error
            return ok, 'ACTUATION_OBSERVATION_PREFIX max_delta_rad=0.025 execution=%s; %s%s' % (
                ok, reason, self._observation_branch_diagnostic(cached))
        except Exception as exc:
            return False, 'ACTUATION_PREFIX_FAILED: %s' % exc

    def _compute_cartesian_plan(self, pose):
        compute_path = self.manipulator.compute_cartesian_path
        waypoints = [deepcopy(pose)]
        eef_step = max(0.0005, float(getattr(self, 'cartesian_eef_step_m', 0.003)))
        jump_threshold = max(0.0, float(getattr(self, 'cartesian_jump_threshold', 0.0)))

        # Noetic's MoveGroupCommander removed jump_threshold from this Python
        # wrapper. Older releases still require it as the third argument.
        try:
            parameters = inspect.signature(compute_path).parameters
        except (TypeError, ValueError):
            parameters = {}
        if 'jump_threshold' in parameters:
            result = compute_path(
                waypoints,
                eef_step,
                jump_threshold,
                avoid_collisions=True,
            )
        else:
            result = compute_path(
                waypoints,
                eef_step,
                avoid_collisions=True,
            )
        if not isinstance(result, tuple) or len(result) < 2:
            return None, 0.0
        first, second = result[0], result[1]
        if isinstance(first, (int, float)):
            return second, float(first)
        return first, float(second)

    def _retime_cartesian_plan(self, plan):
        if not hasattr(self.manipulator, 'retime_trajectory') or not hasattr(self, 'robot'):
            return plan
        try:
            retimed = self.manipulator.retime_trajectory(
                self.robot.get_current_state(),
                plan,
                velocity_scaling_factor=min(
                    1.0, max(0.01, float(getattr(self, 'cartesian_velocity_scaling', 0.20)))
                ),
                acceleration_scaling_factor=min(
                    1.0, max(0.01, float(getattr(self, 'cartesian_acceleration_scaling', 0.30)))
                ),
            )
            if self._plan_success(retimed):
                return retimed
            rospy.logwarn('Cartesian trajectory retiming returned an empty plan; using original timing')
            return plan
        except Exception as exc:
            rospy.logwarn('Cartesian trajectory retiming failed; using original timing: %s', exc)
            return plan

    def plan_and_execute_joint_probe(self, joints, execute=False,
                                    start_joint_positions=None, before_execute=None):
        """Exact joint target for an admitted diagnostic or pregrasp correction.

        The gateway owns the count cap, scene, stage and control provenance.
        Use the gateway's frozen transmitted reference for a command path;
        accepted feedback must remain inside the original .035 rad contract.
        With no reference argument retain fresh-feedback planning. Use
        the same final retiming/reference/CAD/execution chain as strict poses.
        No pose IK, generic go(), gripper command or reusable probe cache.
        """
        self._last_pose_plan = None
        previous_tolerance = None
        try:
            if not self.ready:
                raise ValueError(self.error or 'MoveIt not ready')
            names = tuple(self.manipulator.get_active_joints())
            goal = tuple(float(value) for value in joints)
            if (len(goal) != 6 or len(names) != 6 or len(set(names)) != 6
                    or set(names) != {'Joint%d' % i for i in range(1, 7)}
                    or not all(math.isfinite(v) for v in goal)):
                raise ValueError('JOINT_PROBE_TARGET_INVALID')
            # Public probe targets use canonical arm order, independently of
            # MoveGroup's active-joint enumeration.
            goal_by_name = dict(zip(('Joint%d' % i for i in range(1, 7)), goal))
            state = deepcopy(self.robot.get_current_state())
            if start_joint_positions is not None:
                reference = tuple(float(v) for v in start_joint_positions)
                if len(reference) != 6 or not all(math.isfinite(v) for v in reference):
                    raise ValueError('JOINT_PROBE_REFERENCE_INVALID')
                reference_by_name = dict(zip(('Joint%d' % i for i in range(1, 7)), reference))
                joint_state = state.joint_state
                if (len(joint_state.name) != len(joint_state.position)
                        or len(set(joint_state.name)) != len(joint_state.name)):
                    raise ValueError('JOINT_PROBE_FEEDBACK_INVALID')
                actual = dict(zip(joint_state.name, joint_state.position))
                if any(n not in actual or not math.isfinite(actual[n])
                       or abs(actual[n]-reference_by_name[n]) > .035 for n in names):
                    raise ValueError('JOINT_PROBE_REFERENCE_FOLLOWING_EXCEEDED')
                joint_state.position = [reference_by_name.get(n, p)
                                        for n, p in zip(joint_state.name, joint_state.position)]
                if len(joint_state.velocity) == len(joint_state.name):
                    joint_state.velocity = [0. if n in reference_by_name else v
                                            for n, v in zip(joint_state.name, joint_state.velocity)]
            self.manipulator.set_start_state(state)
            previous_tolerance = float(self.manipulator.get_goal_joint_tolerance())
            if not math.isfinite(previous_tolerance) or previous_tolerance < 0:
                previous_tolerance = None
                raise ValueError('JOINT_PROBE_TOLERANCE_INVALID')
            try:
                self.manipulator.set_goal_joint_tolerance(0.)
                self.manipulator.set_joint_value_target(goal_by_name)
                result = self.manipulator.plan()
                if not self._plan_success(result):
                    raise ValueError('JOINT_PROBE_PLAN_FAILED')
                plan = self._executable_plan(result)
                tr = plan.joint_trajectory
                if (len(tr.joint_names) != 6 or set(tr.joint_names) != set(names)
                        or len(tr.points) < 2
                        or any(len(p.positions) != 6 or not all(math.isfinite(v) for v in p.positions)
                               for p in tr.points)):
                    raise ValueError('JOINT_PROBE_TRAJECTORY_INVALID')
                endpoint = dict(zip(tr.joint_names, tr.points[-1].positions))
                if max(abs(endpoint[j]-goal_by_name[j]) for j in names) > 1e-12:
                    raise ValueError('JOINT_PROBE_GOAL_CHANGED')
                terminal = self._robot_state_after_plan(state, plan)
                from std_msgs.msg import Header
                header = Header()
                header.frame_id = self.manipulator.get_planning_frame()
                pose, reason = self._forward_kinematics_from_state(terminal, header)
                if pose is None:
                    raise ValueError('JOINT_PROBE_FK_FAILED: '+reason)
                pose = deepcopy(getattr(pose, 'pose', pose))
                self._remember_pose_plan(pose, plan, 'strict pose')
            finally:
                self.manipulator.set_goal_joint_tolerance(previous_tolerance)
                previous_tolerance = None
            if not execute:
                return True, 'exact joint probe planned; no motion or reusable execution cache'
            return self._execute_cached_pose_plan(pose, 'bounded exact joint probe',
                required_kind='strict pose', before_execute=before_execute)
        except Exception as exc:
            return False, 'JOINT_PROBE_FAILED: %s' % exc
        finally:
            self._last_pose_plan = None
            self._clear_targets()

    def move_to_joints(self, joints, execute=True):
        if not self.ready:
            return False, self.error or 'MoveIt not ready'
        try:
            self.manipulator.set_joint_value_target(list(joints[:6]))
            if execute:
                # Any arm motion invalidates the start state of a pose plan.
                self._last_pose_plan = None
                ok = self.manipulator.go(wait=True)
                self.manipulator.stop()
                gripper_ok = True
                if len(joints) > 6:
                    self.gripper.set_joint_value_target([joints[6]])
                    gripper_ok = self.gripper.go(wait=True)
                    self.gripper.stop()
                ok = bool(ok) and bool(gripper_ok)
                return ok, 'joint target executed' if ok else 'joint target failed'
            plan_result = self.manipulator.plan()
            ok = self._plan_success(plan_result)
            if len(joints) > 6:
                self.gripper.set_joint_value_target([joints[6]])
                gripper_plan = self.gripper.plan()
                ok = ok and self._plan_success(gripper_plan)
            return ok, 'joint target planned' if ok else 'joint target plan failed'
        except Exception as exc:
            return False, str(exc)

    def get_current_pose(self):
        if not self.ready:
            return None
        return self.manipulator.get_current_pose()

    @staticmethod
    def _planning_now_sec():
        return float(rospy.Time.now().to_sec())

    def _planning_deadline_remaining_sec(self, deadline_sec):
        try:
            value = float(deadline_sec)
        except (TypeError, ValueError, OverflowError):
            return float('-inf')
        if value == 0.0:
            return float('inf')
        if not math.isfinite(value) or value < 0.0:
            return float('-inf')
        try:
            now_sec = float(self._planning_now_sec())
        except (TypeError, ValueError, OverflowError):
            return float('-inf')
        if not math.isfinite(now_sec):
            return float('-inf')
        return value - now_sec

    def check_pose_sequence(
        self,
        targets,
        stage_names=None,
        linear=None,
        deadline_sec=0.0,
    ):
        """Plan a collision-aware pose sequence without moving or caching it."""

        poses = tuple(targets or ())
        names = tuple(stage_names or ())
        linear_flags = tuple(linear or ())
        total_path_cost = 0.0
        max_joint_delta = 0.0
        total_duration = 0.0
        hardware_limiting_joint = ''
        max_stage_duration = 0.0

        def sequence_metrics():
            return {
                'path_cost': total_path_cost,
                'max_delta': max_joint_delta,
                'joint_duration_lower_bound_sec': total_duration,
                # Historical alias retained for existing callers.
                'execution_duration_lower_bound_sec': total_duration,
                'hardware_limiting_joint': hardware_limiting_joint,
            }

        if not self.ready:
            return (
                False,
                'MOVEIT_CHECK_ERROR',
                '',
                sequence_metrics(),
                self.error or 'MoveIt not ready',
            )
        if not poses or len(poses) > 5:
            return (
                False,
                'MOVEIT_CHECK_ERROR',
                '',
                sequence_metrics(),
                'strict pose sequence requires between one and five targets',
            )
        if len(names) != len(poses) or len(linear_flags) != len(poses):
            return (
                False,
                'MOVEIT_CHECK_ERROR',
                '',
                sequence_metrics(),
                'strict pose sequence metadata length does not match targets',
            )
        if not hasattr(self, 'robot') or not hasattr(self.robot, 'get_current_state'):
            return (
                False,
                'MOVEIT_CHECK_ERROR',
                names[0],
                sequence_metrics(),
                'strict pose sequence current robot state is unavailable',
            )

        self._last_pose_plan = None
        try:
            start_state = deepcopy(self.robot.get_current_state())
            if getattr(start_state, 'joint_state', None) is None:
                raise RuntimeError('current robot state has no joint_state')
            for index, (target, stage_name, use_linear) in enumerate(
                zip(poses, names, linear_flags)
            ):
                remaining_sec = self._planning_deadline_remaining_sec(
                    deadline_sec
                )
                if remaining_sec <= 0.0:
                    return (
                        False,
                        'MOVEIT_TIMEOUT',
                        str(stage_name),
                        sequence_metrics(),
                        'strict sequence deadline consumed before %s'
                        % stage_name,
                    )
                pose = getattr(target, 'pose', target)
                if bool(use_linear):
                    plan, reason = self._plan_cartesian_from_start_state(
                        start_state,
                        target,
                    )
                else:
                    if math.isfinite(remaining_sec):
                        plan, reason = self._plan_pose_from_start_state(
                            start_state,
                            pose,
                            planning_time_limit_sec=remaining_sec,
                        )
                    else:
                        plan, reason = self._plan_pose_from_start_state(
                            start_state,
                            pose,
                        )
                if self._planning_deadline_remaining_sec(deadline_sec) <= 0.0:
                    return (
                        False,
                        'MOVEIT_TIMEOUT',
                        str(stage_name),
                        sequence_metrics(),
                        'strict sequence deadline consumed during %s'
                        % stage_name,
                    )
                if plan is None:
                    return (
                        False,
                        'MOVEIT_UNREACHABLE',
                        str(stage_name),
                        sequence_metrics(),
                        'strict sequence %s unreachable: %s'
                        % (stage_name, reason),
                    )
                trajectory = getattr(plan, 'joint_trajectory', None)
                points = list(getattr(trajectory, 'points', []) or [])
                if len(points) < 2:
                    return (
                        False,
                        'MOVEIT_UNREACHABLE',
                        str(stage_name),
                        sequence_metrics(),
                        'strict sequence %s returned an empty or incomplete trajectory'
                        % stage_name,
                    )
                metrics = self._plan_joint_path_metrics(plan)
                stage_duration = float(
                    metrics.get(
                        'joint_duration_lower_bound_sec',
                        metrics.get('execution_duration_lower_bound_sec', 0.0),
                    )
                )
                if not math.isfinite(stage_duration) or stage_duration < 0.0:
                    raise ValueError(
                        'strict sequence %s has non-finite joint duration'
                        % stage_name
                    )
                total_path_cost += max(0.0, float(metrics['path_cost']))
                max_joint_delta = max(
                    max_joint_delta,
                    max(0.0, float(metrics['max_delta'])),
                )
                total_duration += stage_duration
                if stage_duration > max_stage_duration:
                    max_stage_duration = stage_duration
                    hardware_limiting_joint = str(
                        metrics.get('hardware_limiting_joint', '') or ''
                    )
                next_state = self._robot_state_after_plan(start_state, plan)
                if next_state is None:
                    return (
                        False,
                        'MOVEIT_CHECK_ERROR',
                        str(stage_name),
                        sequence_metrics(),
                        'strict sequence %s has no usable terminal joint state'
                        % stage_name,
                    )
                start_state = next_state
            return (
                True,
                '',
                '',
                sequence_metrics(),
                (
                    'strict pose sequence planned stages=%s '
                    'joint_path_cost=%.3f joint_max_delta=%.3f '
                    'joint_duration_lower_bound_sec=%.3f '
                    'execution_duration_lower_bound_sec=%.3f '
                    'hardware_limiting_joint=%s'
                )
                % (
                    ','.join(str(name) for name in names),
                    total_path_cost,
                    max_joint_delta,
                    total_duration,
                    total_duration,
                    hardware_limiting_joint or 'unavailable',
                ),
            )
        except Exception as exc:
            return (
                False,
                'MOVEIT_CHECK_ERROR',
                '',
                sequence_metrics(),
                'strict pose sequence exception: %s' % exc,
            )
        finally:
            self._clear_targets()
            if hasattr(self.manipulator, 'set_start_state_to_current_state'):
                self.manipulator.set_start_state_to_current_state()

    def resolve_free_space_orientations(
        self,
        targets,
        stage_names=None,
        linear=None,
        resolve_orientation=None,
        deadline_sec=0.0,
        _start_state=None,
        _anchor=None,
        _branch_budget=None,
    ):
        """Return deterministic FK-derived orientation seeds without motion.

        For each flagged free-space stage, this resolver samples one bounded,
        deterministic quaternion geodesic between the perception-supplied
        contact orientation and a request-local reachable anchor orientation.
        Every exact sample is solved by collision-aware IK from the same
        virtual joint seed. In cost order, repeatable seeds must also admit
        the exact remaining sequence. Failed suffixes backtrack to the next
        seed within the original deadline and a shared bounded branch count.

        No position-only MoveGroup constraint sampler, cached trajectory, or
        execution API is used.  Unflagged stages retain their exact supplied
        pose and are planned in order only to advance the virtual start state.
        The returned poses remain seeds: callers must rerun analytical
        geometry and strict planning for the exact resolved sequence.
        """

        poses = tuple(targets or ())
        names = tuple(stage_names or ())
        linear_flags = tuple(linear or ())
        resolve_flags = tuple(resolve_orientation or ())
        empty_metrics = {
            'path_cost': 0.0,
            'max_delta': 0.0,
            'max_position_error': 0.0,
        }
        if not self.ready:
            return (
                False,
                'MOVEIT_RESOLVE_ERROR',
                '',
                (),
                empty_metrics,
                self.error or 'MoveIt not ready',
            )
        if not poses or len(poses) > 5:
            return (
                False,
                'MOVEIT_RESOLVE_ERROR',
                '',
                (),
                empty_metrics,
                'orientation resolver requires between one and five targets',
            )
        if (
            len(names) != len(poses)
            or len(linear_flags) != len(poses)
            or len(resolve_flags) != len(poses)
        ):
            return (
                False,
                'MOVEIT_RESOLVE_ERROR',
                '',
                (),
                empty_metrics,
                'orientation resolver metadata length does not match targets',
            )
        if not any(bool(value) for value in resolve_flags) and _start_state is None:
            return (
                False,
                'MOVEIT_RESOLVE_ERROR',
                '',
                (),
                empty_metrics,
                'orientation resolver requires at least one flagged stage',
            )
        for stage_name, use_linear, should_resolve in zip(
            names,
            linear_flags,
            resolve_flags,
        ):
            if bool(use_linear) and bool(should_resolve):
                return (
                    False,
                    'MOVEIT_RESOLVE_ERROR',
                    str(stage_name),
                    (),
                    empty_metrics,
                    (
                        'orientation resolver stage %s cannot be both '
                        'free-space orientation resolution and Cartesian'
                    )
                    % stage_name,
                )
        if not hasattr(self, 'robot') or not hasattr(self.robot, 'get_current_state'):
            return (
                False,
                'MOVEIT_RESOLVE_ERROR',
                names[0],
                (),
                empty_metrics,
                'orientation resolver current robot state is unavailable',
            )

        resolved = []
        total_path_cost = 0.0
        max_joint_delta = 0.0
        max_position_error = 0.0
        orientation_candidates_tested = 0
        max_repeatability_error = 0.0
        free_space_anchor = None
        if _branch_budget is None:
            _branch_budget = [max(1, min(256, int(getattr(
                self, 'orientation_resolution_max_candidates', 96))))]
        self._last_pose_plan = None
        try:
            start_state = deepcopy(self.robot.get_current_state()
                                   if _start_state is None else _start_state)
            if getattr(start_state, 'joint_state', None) is None:
                raise RuntimeError('current robot state has no joint_state')
            if _start_state is None:
                # A near-field request can expire before its final gate audit
                # is committed. Keep the exact read-only solver inputs in
                # rosout so a rejected/expired sequence remains reproducible.
                try:
                    replay_stages = []
                    for target, name, is_linear, free_orientation in zip(
                            poses, names, linear_flags, resolve_flags):
                        item = getattr(target, 'pose', target)
                        header = getattr(target, 'header', None)
                        stamp = getattr(header, 'stamp', None)
                        replay_stages.append(dict(stage=str(name),
                            frame_id=str(getattr(header, 'frame_id', '')),
                            stamp_ns=stamp.to_nsec() if stamp is not None else 0,
                            position_m=[float(getattr(item.position, a)) for a in 'xyz'],
                            quaternion_xyzw=[float(getattr(item.orientation, a)) for a in 'xyzw'],
                            linear_from_prior_state=bool(is_linear),
                            resolve_orientation=bool(free_orientation)))
                    replay = dict(execution_sequence=dict(stages=replay_stages),
                        moveit_input_joint_state=dict(available=True,
                            name=list(start_state.joint_state.name),
                            position_rad=list(start_state.joint_state.position)),
                        deadline_sec=float(deadline_sec), planning_only=True)
                    rospy.loginfo('Orientation resolver input: %s',
                                  json.dumps(replay, allow_nan=False, separators=(',', ':')))
                except (AttributeError, TypeError, ValueError) as exc:
                    rospy.logwarn('Orientation resolver input recording unavailable: %s', exc)
            if self._planning_deadline_remaining_sec(deadline_sec) <= 0.0:
                return (
                    False,
                    'MOVEIT_TIMEOUT',
                    str(names[0]),
                    (),
                    empty_metrics,
                    'orientation resolver deadline consumed before initial FK',
                )
            if _anchor is None:
                initial_fk, initial_fk_reason = self._forward_kinematics_from_state(
                    start_state, getattr(poses[0], 'header', None))
            else:
                initial_fk, initial_fk_reason = deepcopy(_anchor), 'branch anchor'
            if initial_fk is None:
                return (
                    False,
                    'MOVEIT_RESOLVE_ERROR',
                    str(names[0]),
                    (),
                    empty_metrics,
                    'orientation resolver initial FK failed: %s'
                    % initial_fk_reason,
                )
            free_space_anchor = initial_fk
            for stage_index, (target, stage_name, use_linear, should_resolve) in enumerate(zip(
                poses,
                names,
                linear_flags,
                resolve_flags,
            )):
                accepted_suffix = []
                failed_suffix = []
                remaining_sec = self._planning_deadline_remaining_sec(
                    deadline_sec
                )
                if remaining_sec <= 0.0:
                    return (
                        False,
                        'MOVEIT_TIMEOUT',
                        str(stage_name),
                        tuple(resolved),
                        {
                            'path_cost': total_path_cost,
                            'max_delta': max_joint_delta,
                            'max_position_error': max_position_error,
                        },
                        'orientation resolver deadline consumed before %s'
                        % stage_name,
                    )
                pose = getattr(target, 'pose', target)
                if bool(should_resolve):
                    if _branch_budget[0] <= 0:
                        return (False, 'MOVEIT_SEARCH_EXHAUSTED', str(stage_name),
                                tuple(resolved), empty_metrics,
                                'bounded orientation branch budget exhausted')
                    if bool(use_linear):
                        return (
                            False,
                            'MOVEIT_RESOLVE_ERROR',
                            str(stage_name),
                            tuple(resolved),
                            {
                                'path_cost': total_path_cost,
                                'max_delta': max_joint_delta,
                                'max_position_error': max_position_error,
                            },
                            (
                                'orientation resolver stage %s cannot be both '
                                'free-space orientation resolution and Cartesian'
                            )
                            % stage_name,
                        )
                    def accept_seed(candidate_seed):
                        if _branch_budget[0] <= 0:
                            return False, 'MOVEIT_SEARCH_EXHAUSTED', 'bounded orientation branch budget exhausted'
                        _branch_budget[0] -= 1
                        if stage_index + 1 == len(poses):
                            return True, '', ''
                        # Only the first free-space stage establishes the
                        # request-local anchor. Lift uses the pregrasp anchor,
                        # never a newly measured or unrelated current pose.
                        next_anchor = (candidate_seed['resolved_target']
                                       if _anchor is None and free_space_anchor is initial_fk
                                       else free_space_anchor)
                        suffix = self.resolve_free_space_orientations(
                            poses[stage_index + 1:], names[stage_index + 1:],
                            linear_flags[stage_index + 1:], resolve_flags[stage_index + 1:],
                            deadline_sec=deadline_sec,
                            _start_state=candidate_seed['terminal_state'],
                            _anchor=next_anchor, _branch_budget=_branch_budget)
                        if suffix[0]:
                            accepted_suffix.append(suffix)
                            return True, '', ''
                        failed_suffix[:] = [suffix]
                        return False, suffix[1], suffix[5]

                    if math.isfinite(remaining_sec):
                        seed, code, reason = (
                            self._resolve_orientation_from_state(
                                start_state,
                                target,
                                free_space_anchor,
                                deadline_sec=deadline_sec,
                                accept_seed=accept_seed,
                            )
                        )
                    else:
                        seed, code, reason = (
                            self._resolve_orientation_from_state(
                                start_state,
                                target,
                                free_space_anchor,
                                accept_seed=accept_seed,
                            )
                        )
                    if seed is None:
                        if failed_suffix and code not in ('MOVEIT_TIMEOUT', 'MOVEIT_SEARCH_EXHAUSTED'):
                            failure = failed_suffix[-1]
                            return (False, failure[1], failure[2], tuple(resolved),
                                    failure[4], '%s; all ranked branches rejected (%s)'
                                    % (failure[5], reason))
                        return (
                            False,
                            ('MOVEIT_UNREACHABLE' if code == 'MOVEIT_SEARCH_EXHAUSTED'
                             else str(code or 'MOVEIT_UNREACHABLE')),
                            str(stage_name),
                            tuple(resolved),
                            {
                                'path_cost': total_path_cost,
                                'max_delta': max_joint_delta,
                                'max_position_error': max_position_error,
                            },
                            'orientation resolver %s unreachable: %s'
                            % (stage_name, reason),
                        )
                    resolved_target = seed['resolved_target']
                    next_state = seed['terminal_state']
                    stage_metrics = {
                        'path_cost': float(seed['path_cost']),
                        'max_delta': float(seed['max_delta']),
                    }
                    position_error = float(seed['position_error'])
                    orientation_candidates_tested += int(
                        seed['candidates_tested']
                    )
                    max_repeatability_error = max(
                        max_repeatability_error,
                        float(seed['repeatability_error']),
                    )
                    if free_space_anchor is initial_fk:
                        free_space_anchor = deepcopy(resolved_target)
                else:
                    if bool(use_linear):
                        plan, reason = self._plan_cartesian_from_start_state(
                            start_state,
                            target,
                        )
                    else:
                        if math.isfinite(remaining_sec):
                            plan, reason = self._plan_pose_from_start_state(
                                start_state,
                                pose,
                                planning_time_limit_sec=remaining_sec,
                            )
                        else:
                            plan, reason = self._plan_pose_from_start_state(
                                start_state,
                                pose,
                            )
                    if (
                        self._planning_deadline_remaining_sec(deadline_sec)
                        <= 0.0
                    ):
                        return (
                            False,
                            'MOVEIT_TIMEOUT',
                            str(stage_name),
                            tuple(resolved),
                            {
                                'path_cost': total_path_cost,
                                'max_delta': max_joint_delta,
                                'max_position_error': max_position_error,
                            },
                            'orientation resolver deadline consumed during %s'
                            % stage_name,
                        )
                    if plan is None:
                        return (
                            False,
                            'MOVEIT_UNREACHABLE',
                            str(stage_name),
                            tuple(resolved),
                            {
                                'path_cost': total_path_cost,
                                'max_delta': max_joint_delta,
                                'max_position_error': max_position_error,
                            },
                            'orientation resolver %s unreachable: %s'
                            % (stage_name, reason),
                        )
                    stage_metrics = self._plan_joint_path_metrics(plan)
                    next_state = self._robot_state_after_plan(
                        start_state,
                        plan,
                    )
                    if next_state is None:
                        return (
                            False,
                            'MOVEIT_RESOLVE_ERROR',
                            str(stage_name),
                            tuple(resolved),
                            {
                                'path_cost': total_path_cost,
                                'max_delta': max_joint_delta,
                                'max_position_error': max_position_error,
                            },
                            (
                                'orientation resolver %s has no usable '
                                'terminal joint state'
                            )
                            % stage_name,
                        )
                    resolved_target = deepcopy(target)
                    position_error = 0.0

                total_path_cost += max(
                    0.0,
                    float(stage_metrics['path_cost']),
                )
                max_joint_delta = max(
                    max_joint_delta,
                    max(0.0, float(stage_metrics['max_delta'])),
                )
                if not math.isfinite(position_error):
                    raise RuntimeError(
                        'orientation resolver FK position error is non-finite'
                    )
                max_position_error = max(
                    max_position_error,
                    position_error,
                )
                resolved.append(resolved_target)
                start_state = next_state
                if accepted_suffix:
                    suffix = accepted_suffix[-1]
                    resolved.extend(suffix[3])
                    total_path_cost += suffix[4]['path_cost']
                    max_joint_delta = max(max_joint_delta, suffix[4]['max_delta'])
                    max_position_error = max(max_position_error, suffix[4]['max_position_error'])
                    orientation_candidates_tested += suffix[4].get('orientation_candidates_tested', 0)
                    max_repeatability_error = max(max_repeatability_error,
                                                   suffix[4].get('max_repeatability_error', 0.0))
                    break

            return (
                True,
                '',
                '',
                tuple(resolved),
                {
                    'path_cost': total_path_cost,
                    'max_delta': max_joint_delta,
                    'max_position_error': max_position_error,
                    'orientation_candidates_tested': orientation_candidates_tested,
                    'max_repeatability_error': max_repeatability_error,
                },
                (
                    'free-space orientation seeds resolved stages=%s '
                    'policy=deterministic_geodesic_collision_ik '
                    'candidates_tested=%d '
                    'max_repeatability_error=%.9f '
                    'joint_path_cost=%.3f joint_max_delta=%.3f '
                    'max_fk_position_error=%.6f'
                )
                % (
                    ','.join(
                        str(name)
                        for name, flag in zip(names, resolve_flags)
                        if bool(flag)
                    ),
                    orientation_candidates_tested,
                    max_repeatability_error,
                    total_path_cost,
                    max_joint_delta,
                    max_position_error,
                ),
            )
        except Exception as exc:
            return (
                False,
                'MOVEIT_RESOLVE_ERROR',
                '',
                tuple(resolved),
                {
                    'path_cost': total_path_cost,
                    'max_delta': max_joint_delta,
                    'max_position_error': max_position_error,
                },
                'orientation resolver exception: %s' % exc,
            )
        finally:
            self._clear_targets()
            if hasattr(self.manipulator, 'set_start_state_to_current_state'):
                self.manipulator.set_start_state_to_current_state()

    def _resolve_orientation_from_state(
        self,
        start_state,
        target,
        anchor_target,
        deadline_sec=0.0,
        accept_seed=None,
    ):
        """Return the first repeatable seed whose bounded suffix also passes."""

        target_pose = getattr(target, 'pose', target)
        anchor_pose = getattr(anchor_target, 'pose', anchor_target)
        try:
            samples = self._orientation_geodesic_samples(
                target_pose.orientation,
                anchor_pose.orientation,
                max_step_rad=max(
                    1e-3,
                    float(
                        getattr(
                            self,
                            'orientation_resolution_step_rad',
                            getattr(
                                self,
                                'cached_plan_orientation_tolerance_rad',
                                0.02,
                            ),
                        )
                    ),
                ),
                max_candidates=max(
                    2,
                    int(
                        getattr(
                            self,
                            'orientation_resolution_max_candidates',
                            96,
                        )
                    ),
                ),
            )
        except Exception as exc:
            return (
                None,
                'MOVEIT_RESOLVE_ERROR',
                'deterministic orientation samples are invalid: %s' % exc,
            )

        successes = []
        attempted = 0
        for fraction, quaternion in samples:
            remaining_sec = self._planning_deadline_remaining_sec(
                deadline_sec
            )
            if remaining_sec <= 0.0:
                return (
                    None,
                    'MOVEIT_TIMEOUT',
                    'orientation resolver deadline consumed before IK sample',
                )
            attempted += 1
            candidate_target = deepcopy(target)
            candidate_pose = getattr(
                candidate_target,
                'pose',
                candidate_target,
            )
            self._assign_quaternion(candidate_pose.orientation, quaternion)
            if math.isfinite(remaining_sec):
                ik_state, _ik_reason = self._inverse_kinematics_from_state(
                    start_state,
                    candidate_target,
                    timeout_sec=remaining_sec,
                )
            else:
                ik_state, _ik_reason = self._inverse_kinematics_from_state(
                    start_state,
                    candidate_target,
                )
            if self._planning_deadline_remaining_sec(deadline_sec) <= 0.0:
                return (
                    None,
                    'MOVEIT_TIMEOUT',
                    'orientation resolver deadline consumed during IK sample',
                )
            if ik_state is None:
                continue
            try:
                metrics = self._joint_state_delta_metrics(
                    start_state,
                    ik_state,
                )
            except Exception:
                continue
            successes.append(
                (
                    float(metrics['path_cost']),
                    float(metrics['max_delta']),
                    float(fraction),
                    candidate_target,
                    ik_state,
                )
            )
        if not successes:
            return (
                None,
                'MOVEIT_UNREACHABLE',
                (
                    'deterministic geodesic collision-aware IK found no '
                    'solution across %d exact orientations'
                )
                % attempted,
            )
        successes.sort(key=lambda item: (item[0], item[1], item[2]))
        last_code, last_reason = 'MOVEIT_UNREACHABLE', 'no accepted orientation seed'
        for selection in successes:
            seed, code, reason = self._validate_orientation_seed(
                start_state, target, selection, attempted, deadline_sec)
            if seed is None:
                if code == 'MOVEIT_TIMEOUT':
                    return None, code, reason
                last_code, last_reason = code, reason
                continue
            if self._planning_deadline_remaining_sec(deadline_sec) <= 0.0:
                return None, 'MOVEIT_TIMEOUT', 'orientation resolver deadline consumed during seed validation'
            if accept_seed is not None:
                accepted, code, reason = accept_seed(seed)
                if not accepted:
                    if code in ('MOVEIT_TIMEOUT', 'MOVEIT_SEARCH_EXHAUSTED'):
                        return None, code, reason
                    last_code, last_reason = code, reason
                    continue
            return seed, '', 'deterministic geodesic IK accepted repeatable full-sequence seed'
        return None, last_code, '%s; exhausted %d ranked IK seeds' % (last_reason, len(successes))

    def _validate_orientation_seed(self, start_state, target, selection,
                                   attempted, deadline_sec):
        """Repeat IK and measure FK; never relax either check for a fallback."""
        target_pose = getattr(target, 'pose', target)
        path_cost, max_delta, fraction, selected_target, selected_state = (
            selection
        )
        remaining_sec = self._planning_deadline_remaining_sec(deadline_sec)
        if remaining_sec <= 0.0:
            return (
                None,
                'MOVEIT_TIMEOUT',
                'orientation resolver deadline consumed before repeatability IK',
            )
        if math.isfinite(remaining_sec):
            repeated_state, repeated_reason = (
                self._inverse_kinematics_from_state(
                    start_state,
                    selected_target,
                    timeout_sec=remaining_sec,
                )
            )
        else:
            repeated_state, repeated_reason = (
                self._inverse_kinematics_from_state(
                    start_state,
                    selected_target,
                )
            )
        if self._planning_deadline_remaining_sec(deadline_sec) <= 0.0:
            return (
                None,
                'MOVEIT_TIMEOUT',
                'orientation resolver deadline consumed during repeatability IK',
            )
        if repeated_state is None:
            return (
                None,
                'MOVEIT_RESOLVE_NONDETERMINISTIC',
                'selected IK seed did not repeat: %s' % repeated_reason,
            )
        try:
            repeatability_error = self._joint_state_max_difference(
                selected_state,
                repeated_state,
            )
        except Exception as exc:
            return (
                None,
                'MOVEIT_RESOLVE_NONDETERMINISTIC',
                'selected IK repeatability comparison failed: %s' % exc,
            )
        tolerance = max(
            0.0,
            float(
                getattr(
                    self,
                    'orientation_resolution_repeatability_tolerance_rad',
                    1e-6,
                )
            ),
        )
        if repeatability_error > tolerance:
            return (
                None,
                'MOVEIT_RESOLVE_NONDETERMINISTIC',
                (
                    'selected IK seed repeatability error %.9frad exceeds '
                    '%.9frad'
                )
                % (repeatability_error, tolerance),
            )
        fk_target, fk_reason = self._forward_kinematics_from_state(
            selected_state,
            getattr(target, 'header', None),
        )
        if fk_target is None:
            return (
                None,
                'MOVEIT_RESOLVE_ERROR',
                'selected deterministic IK FK failed: %s' % fk_reason,
            )
        fk_pose = getattr(fk_target, 'pose', fk_target)
        position_error = self._pose_position_distance(target_pose, fk_pose)
        if not math.isfinite(position_error):
            return (
                None,
                'MOVEIT_RESOLVE_ERROR',
                'selected deterministic IK FK position error is non-finite',
            )
        resolved_target = deepcopy(target)
        resolved_pose = getattr(resolved_target, 'pose', resolved_target)
        resolved_pose.orientation = deepcopy(fk_pose.orientation)
        return (
            {
                'resolved_target': resolved_target,
                'terminal_state': selected_state,
                'path_cost': path_cost,
                'max_delta': max_delta,
                'position_error': position_error,
                'candidates_tested': attempted,
                'repeatability_error': repeatability_error,
                'geodesic_fraction': fraction,
            },
            '',
            (
                'deterministic geodesic IK selected fraction=%.9f '
                'from %d samples'
            )
            % (fraction, attempted),
        )

    def _inverse_kinematics_from_state(
        self,
        start_state,
        target,
        timeout_sec=None,
    ):
        """Solve one exact collision-aware IK request from an explicit seed."""

        try:
            from geometry_msgs.msg import PoseStamped
            from moveit_msgs.msg import MoveItErrorCodes
            from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

            service_name = '/compute_ik'
            rospy.wait_for_service(service_name, timeout=0.5)
            service = getattr(self, '_ik_service', None)
            if service is None:
                service = rospy.ServiceProxy(service_name, GetPositionIK)
                self._ik_service = service
            request = GetPositionIKRequest()
            request.ik_request.group_name = str(self.manipulator_group)
            request.ik_request.robot_state = deepcopy(start_state)
            request.ik_request.avoid_collisions = True
            if hasattr(self.manipulator, 'get_end_effector_link'):
                request.ik_request.ik_link_name = str(
                    self.manipulator.get_end_effector_link() or ''
                )
            if hasattr(target, 'header') and hasattr(target, 'pose'):
                request.ik_request.pose_stamped = deepcopy(target)
            else:
                pose_stamped = PoseStamped()
                if hasattr(self.manipulator, 'get_planning_frame'):
                    pose_stamped.header.frame_id = str(
                        self.manipulator.get_planning_frame() or ''
                    )
                pose_stamped.pose = deepcopy(getattr(target, 'pose', target))
                request.ik_request.pose_stamped = pose_stamped
            configured_timeout_sec = max(
                1e-3,
                float(
                    getattr(
                        self,
                        'orientation_resolution_ik_timeout_sec',
                        0.05,
                    )
                ),
            )
            if timeout_sec is not None:
                configured_timeout_sec = min(
                    configured_timeout_sec,
                    max(1e-3, float(timeout_sec)),
                )
            request.ik_request.timeout = rospy.Duration.from_sec(
                configured_timeout_sec
            )
            response = service(request)
            error_code = int(
                getattr(getattr(response, 'error_code', None), 'val', 0) or 0
            )
            if error_code != int(MoveItErrorCodes.SUCCESS):
                return None, 'MoveIt IK returned error code %d' % error_code
            solution = deepcopy(getattr(response, 'solution', None))
            if getattr(solution, 'joint_state', None) is None:
                return None, 'MoveIt IK returned no joint state'
            self._joint_state_delta_metrics(start_state, solution)
            return solution, 'collision-aware IK resolved'
        except Exception as exc:
            return None, 'MoveIt IK failed: %s' % exc

    @classmethod
    def _orientation_geodesic_samples(
        cls,
        first_orientation,
        second_orientation,
        max_step_rad,
        max_candidates,
    ):
        first = cls._normalized_quaternion(first_orientation)
        second = cls._normalized_quaternion(second_orientation)
        dot = sum(a * b for a, b in zip(first, second))
        if dot < 0.0:
            second = tuple(-value for value in second)
            dot = -dot
        dot = min(1.0, max(-1.0, dot))
        angle = 2.0 * math.acos(dot)
        sample_count = max(
            2,
            min(
                int(max_candidates),
                int(math.ceil(angle / float(max_step_rad))) + 1,
            ),
        )
        output = []
        for index in range(sample_count):
            fraction = float(index) / float(sample_count - 1)
            output.append(
                (
                    fraction,
                    cls._quaternion_slerp(first, second, fraction),
                )
            )
        return tuple(output)

    @staticmethod
    def _normalized_quaternion(orientation):
        values = tuple(
            float(getattr(orientation, name))
            for name in ('x', 'y', 'z', 'w')
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError('orientation contains non-finite values')
        norm = math.sqrt(sum(value * value for value in values))
        if norm <= 1e-12:
            raise ValueError('orientation quaternion is degenerate')
        values = tuple(value / norm for value in values)
        if values[3] < 0.0:
            values = tuple(-value for value in values)
        return values

    @staticmethod
    def _quaternion_slerp(first, second, fraction):
        dot = sum(a * b for a, b in zip(first, second))
        if dot < 0.0:
            second = tuple(-value for value in second)
            dot = -dot
        dot = min(1.0, max(-1.0, dot))
        fraction = min(1.0, max(0.0, float(fraction)))
        if dot > 0.9995:
            output = tuple(
                a + fraction * (b - a)
                for a, b in zip(first, second)
            )
        else:
            angle = math.acos(dot)
            sine = math.sin(angle)
            output = tuple(
                (
                    math.sin((1.0 - fraction) * angle) / sine * a
                    + math.sin(fraction * angle) / sine * b
                )
                for a, b in zip(first, second)
            )
        norm = math.sqrt(sum(value * value for value in output))
        if norm <= 1e-12:
            raise ValueError('slerp produced a degenerate quaternion')
        output = tuple(value / norm for value in output)
        if output[3] < 0.0:
            output = tuple(-value for value in output)
        return output

    @staticmethod
    def _assign_quaternion(orientation, values):
        for name, value in zip(('x', 'y', 'z', 'w'), values):
            setattr(orientation, name, float(value))

    @staticmethod
    def _joint_state_positions(state):
        joint_state = getattr(state, 'joint_state', None)
        if joint_state is None:
            raise ValueError('robot state has no joint state')
        names = [str(name) for name in list(joint_state.name or [])]
        positions = [float(value) for value in list(joint_state.position or [])]
        if not names or len(names) != len(positions):
            raise ValueError('robot joint names and positions are inconsistent')
        if len(set(names)) != len(names):
            raise ValueError('robot joint names are not unique')
        if not all(math.isfinite(value) for value in positions):
            raise ValueError('robot joint positions contain non-finite values')
        return names, positions

    @classmethod
    def _joint_state_delta_metrics(cls, first_state, second_state):
        first_names, first_positions = cls._joint_state_positions(first_state)
        second_names, second_positions = cls._joint_state_positions(second_state)
        second_by_name = dict(zip(second_names, second_positions))
        common = [
            (name, position, second_by_name[name])
            for name, position in zip(first_names, first_positions)
            if name in second_by_name
        ]
        if not common:
            raise ValueError('robot states have no common joints')
        deltas = [
            float(second) - float(first)
            for _name, first, second in common
        ]
        return {
            'path_cost': math.sqrt(sum(value * value for value in deltas)),
            'max_delta': max(abs(value) for value in deltas),
        }

    @classmethod
    def _joint_state_max_difference(cls, first_state, second_state):
        first_names, first_positions = cls._joint_state_positions(first_state)
        second_names, second_positions = cls._joint_state_positions(second_state)
        second_by_name = dict(zip(second_names, second_positions))
        if set(first_names) != set(second_names):
            raise ValueError('repeated IK joint name sets differ')
        return max(
            abs(float(position) - float(second_by_name[name]))
            for name, position in zip(first_names, first_positions)
        )

    def _plan_pose_from_start_state(
        self,
        start_state,
        pose,
        planning_time_limit_sec=None,
    ):
        previous_planning_time = None
        if hasattr(self.manipulator, 'get_planning_time'):
            previous_planning_time = self.manipulator.get_planning_time()
        if hasattr(self.manipulator, 'set_planning_time'):
            planning_time = max(
                0.05,
                float(getattr(self, 'strict_pose_planning_time', 0.25)),
            )
            if planning_time_limit_sec is not None:
                planning_time = min(
                    planning_time,
                    max(1e-3, float(planning_time_limit_sec)),
                )
            self.manipulator.set_planning_time(
                planning_time
            )
        try:
            if not hasattr(self.manipulator, 'set_start_state'):
                return None, 'MoveIt start-state API is unavailable'
            self.manipulator.set_start_state(deepcopy(start_state))
            self.manipulator.set_pose_target(pose)
            result = self.manipulator.plan()
            if not self._plan_success(result):
                return None, 'strict pose planning failed'
            return self._executable_plan(result), 'planned'
        finally:
            self._clear_targets()
            if (
                previous_planning_time is not None
                and hasattr(self.manipulator, 'set_planning_time')
            ):
                self.manipulator.set_planning_time(previous_planning_time)

    def _plan_position_from_start_state(self, start_state, pose):
        previous_planning_time = None
        if hasattr(self.manipulator, 'get_planning_time'):
            previous_planning_time = self.manipulator.get_planning_time()
        if hasattr(self.manipulator, 'set_planning_time'):
            resolver_planning_time = getattr(
                self,
                'orientation_resolution_planning_time',
                previous_planning_time,
            )
            if resolver_planning_time is None:
                resolver_planning_time = getattr(
                    self,
                    'strict_pose_planning_time',
                    0.25,
                )
            self.manipulator.set_planning_time(
                max(0.05, float(resolver_planning_time))
            )
        try:
            if not hasattr(self.manipulator, 'set_start_state'):
                return None, 'MoveIt start-state API is unavailable'
            if not hasattr(self.manipulator, 'set_position_target'):
                return None, 'MoveIt position-target API is unavailable'
            self.manipulator.set_start_state(deepcopy(start_state))
            position = pose.position
            self.manipulator.set_position_target(
                [
                    float(position.x),
                    float(position.y),
                    float(position.z),
                ]
            )
            result = self.manipulator.plan()
            if not self._plan_success(result):
                return None, 'position-only planning failed'
            return self._executable_plan(result), 'position-only planned'
        finally:
            self._clear_targets()
            if (
                previous_planning_time is not None
                and hasattr(self.manipulator, 'set_planning_time')
            ):
                self.manipulator.set_planning_time(previous_planning_time)

    def _forward_kinematics_from_state(self, state, header=None):
        try:
            from moveit_msgs.msg import MoveItErrorCodes
            from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest

            service_name = '/compute_fk'
            rospy.wait_for_service(service_name, timeout=0.5)
            service = getattr(self, '_fk_service', None)
            if service is None:
                service = rospy.ServiceProxy(service_name, GetPositionFK)
                self._fk_service = service
            request = GetPositionFKRequest()
            if header is not None:
                request.header = deepcopy(header)
            if hasattr(self.manipulator, 'get_end_effector_link'):
                link_name = str(
                    self.manipulator.get_end_effector_link() or ''
                )
            else:
                link_name = ''
            if not link_name:
                return None, 'MoveIt end-effector link is unavailable'
            request.fk_link_names = [link_name]
            request.robot_state = deepcopy(state)
            response = service(request)
            error_code = int(
                getattr(getattr(response, 'error_code', None), 'val', 0) or 0
            )
            if error_code != int(MoveItErrorCodes.SUCCESS):
                return None, 'MoveIt FK returned error code %d' % error_code
            poses = list(getattr(response, 'pose_stamped', []) or [])
            if len(poses) != 1:
                return None, 'MoveIt FK did not return exactly one tool pose'
            return poses[0], 'FK resolved'
        except Exception as exc:
            return None, 'MoveIt FK failed: %s' % exc

    def _plan_cartesian_from_start_state(self, start_state, target):
        try:
            from moveit_msgs.srv import GetCartesianPath, GetCartesianPathRequest

            service_name = '/compute_cartesian_path'
            rospy.wait_for_service(service_name, timeout=0.5)
            service = getattr(self, '_cartesian_path_service', None)
            if service is None:
                service = rospy.ServiceProxy(service_name, GetCartesianPath)
                self._cartesian_path_service = service
            request = GetCartesianPathRequest()
            header = getattr(target, 'header', None)
            if header is not None:
                request.header = deepcopy(header)
            request.start_state = deepcopy(start_state)
            request.group_name = str(
                getattr(self, 'manipulator_group', '') or 'alicia'
            )
            if hasattr(self.manipulator, 'get_end_effector_link'):
                request.link_name = str(
                    self.manipulator.get_end_effector_link() or ''
                )
            request.waypoints = [deepcopy(getattr(target, 'pose', target))]
            request.max_step = max(
                1e-5,
                float(getattr(self, 'cartesian_eef_step_m', 0.003)),
            )
            request.jump_threshold = max(
                0.0,
                float(getattr(self, 'cartesian_jump_threshold', 0.0)),
            )
            request.avoid_collisions = True
            response = service(request)
            fraction = float(getattr(response, 'fraction', 0.0) or 0.0)
            minimum = max(
                0.0,
                min(1.0, float(getattr(self, 'cartesian_min_fraction', 0.98))),
            )
            plan = getattr(response, 'solution', None)
            if fraction + 1e-9 < minimum or not self._plan_success(plan):
                return (
                    None,
                    'Cartesian planning fraction %.3f is below %.3f'
                    % (fraction, minimum),
                )
            return plan, 'Cartesian path fraction %.3f' % fraction
        except Exception as exc:
            return None, 'Cartesian planning failed: %s' % exc

    @staticmethod
    def _robot_state_after_plan(start_state, plan):
        trajectory = getattr(plan, 'joint_trajectory', None)
        joint_names = list(getattr(trajectory, 'joint_names', []) or [])
        points = list(getattr(trajectory, 'points', []) or [])
        terminal = list(
            getattr(points[-1], 'positions', []) or ()
        ) if points else []
        if not joint_names or len(joint_names) != len(terminal):
            return None
        state = deepcopy(start_state)
        joint_state = getattr(state, 'joint_state', None)
        if joint_state is None:
            return None
        state_names = list(getattr(joint_state, 'name', []) or [])
        state_positions = list(getattr(joint_state, 'position', []) or [])
        if len(state_names) != len(state_positions):
            return None
        by_name = {
            str(name): float(position)
            for name, position in zip(state_names, state_positions)
        }
        for name, position in zip(joint_names, terminal):
            value = float(position)
            if not math.isfinite(value):
                return None
            by_name[str(name)] = value
        joint_state.name = list(by_name)
        joint_state.position = [by_name[name] for name in joint_state.name]
        joint_state.velocity = []
        joint_state.effort = []
        return state

    def _attempt_pose_target(self, pose, execute, planning_time=None, plan_kind='strict pose'):
        previous_planning_time = None
        if planning_time is not None and hasattr(self.manipulator, 'set_planning_time'):
            if hasattr(self.manipulator, 'get_planning_time'):
                previous_planning_time = self.manipulator.get_planning_time()
            self.manipulator.set_planning_time(max(0.05, float(planning_time)))
        try:
            if (not execute and plan_kind == 'strict pose'
                    and getattr(self, 'observation_branch_hints_enabled', False)):
                return self._plan_observation_with_branch_hint(pose)
            self.manipulator.set_pose_target(pose)
            return self._run_current_target(execute, pose, plan_kind)
        finally:
            self._clear_targets()
            if previous_planning_time is not None:
                self.manipulator.set_planning_time(previous_planning_time)

    def _observation_branch_signature(self):
        """Invalidate hints when the model, group, frame, or tool changes."""
        descriptions = [rospy.get_param(name, '') for name in (
            '/robot_description', '/robot_description_semantic')]
        if not all(isinstance(value, str) and value.strip() for value in descriptions):
            raise ValueError('OBSERVATION_BRANCH_MODEL_UNAVAILABLE')
        names = tuple(self.manipulator.get_active_joints())
        if len(names) != 6 or len(set(names)) != 6:
            raise ValueError('OBSERVATION_BRANCH_REQUIRES_SIX_ARM_JOINTS')
        metadata = [self.manipulator_group, self.manipulator.get_planning_frame(),
                    self.manipulator.get_end_effector_link(), names]
        return hashlib.sha256(json.dumps(
            [descriptions, metadata], separators=(',', ':'),
        ).encode('utf-8')).hexdigest(), names

    def _observation_branch_context(self, pose):
        # Exact floating-point pose components, not the deliberately looser
        # ordinary cached-trajectory matching tolerances. Quaternion signs are
        # left distinct too: a miss may replan, but never borrows another pose.
        xyz = self._pose_xyz_tuple(pose)
        q = tuple(float(getattr(pose.orientation, name)) for name in ('x', 'y', 'z', 'w'))
        if not self._finite_tuple(xyz, 3) or not self._finite_tuple(q, 4) or sum(v*v for v in q) <= 1e-18:
            raise ValueError('OBSERVATION_BRANCH_POSE_INVALID')
        signature, names = self._observation_branch_signature()
        clock = getattr(self, '_observation_branch_clock', time.monotonic)
        now = float(clock())
        if not math.isfinite(now):
            raise ValueError('OBSERVATION_BRANCH_CLOCK_INVALID')
        cache = getattr(self, '_observation_branch_hints', None)
        if cache is None:
            cache = self._observation_branch_hints = OrderedDict()
        for key, hint in list(cache.items()):
            age = now - hint['created_sec']
            if (hint['signature'] != signature or age < 0.0
                    or age > self.OBSERVATION_BRANCH_HINT_MAX_AGE_SEC):
                del cache[key]
        key = (signature, tuple(xyz), q)
        return cache, key, names, now

    def _observation_branch_fk_check(self, state, pose, stage='terminal'):
        from std_msgs.msg import Header
        from tf.transformations import (
            euler_from_quaternion, quaternion_inverse, quaternion_multiply,
        )
        header = Header()
        header.frame_id = str(self.manipulator.get_planning_frame())
        if not header.frame_id:
            raise ValueError('OBSERVATION_BRANCH_FRAME_UNAVAILABLE')
        fk, reason = self._forward_kinematics_from_state(state, header)
        if fk is None:
            raise ValueError('OBSERVATION_BRANCH_FK_FAILED: ' + reason)
        if hasattr(fk, 'header') and str(fk.header.frame_id) != header.frame_id:
            raise ValueError('OBSERVATION_BRANCH_FK_FRAME_MISMATCH')
        measured = getattr(fk, 'pose', fk)
        position_error = self._pose_position_distance(measured, pose)
        actual_q = self._normalized_quaternion(measured.orientation)
        target_q = self._normalized_quaternion(pose.orientation)
        orientation_error = self._quaternion_angle(actual_q, target_q)
        # MoveGroup's scalar orientation tolerance populates three separate
        # intrinsic XYZ Euler-axis bounds, not a bound on the total SO(3) angle. Keep
        # those semantics and the existing cache's angular match independently.
        axis_errors = tuple(abs(float(v)) for v in euler_from_quaternion(
            quaternion_multiply(quaternion_inverse(target_q), actual_q), axes='rxyz'))
        # A fixed joint goal must still satisfy the original complete pose.
        # Never substitute the FK orientation or relax current MoveIt bounds.
        position_tolerance = min(float(self.manipulator.get_goal_position_tolerance()),
                                 float(self.cached_plan_position_tolerance_m))
        orientation_tolerance = float(self.manipulator.get_goal_orientation_tolerance())
        cached_angle_tolerance = float(self.cached_plan_orientation_tolerance_rad)
        if (not all(math.isfinite(v) and v >= 0.0 for v in (
                position_error, orientation_error, position_tolerance,
                orientation_tolerance, cached_angle_tolerance) + axis_errors)
                or position_error > position_tolerance
                or max(axis_errors) > orientation_tolerance
                or orientation_error > cached_angle_tolerance):
            raise ValueError('OBSERVATION_BRANCH_FK_TARGET_MISMATCH: stage=%s position=%.9fm orientation=%.9frad'
                             % (stage, position_error, orientation_error))

    def _plan_observation_with_branch_hint(self, pose):
        """Fresh collision planning to a previously checked joint branch.

        Other candidate checks and a consumed actuation prefix may discard
        _last_pose_plan, but must not silently choose another IK branch for
        the same exact observation target. A hint is not a trajectory cache:
        current robot state and collision scene are used on every invocation.
        """
        self._last_pose_plan = None
        cache, key, names, now = self._observation_branch_context(pose)
        hint = cache.get(key)
        previous_joint_tolerance = None
        try:
            description = rospy.get_param('/robot_description', '')
            if self._observation_branch_signature()[0] != key[0]:
                raise ValueError('OBSERVATION_BRANCH_MODEL_CHANGED')
            start_state = deepcopy(self.robot.get_current_state())
            state_names, state_positions = self._joint_state_positions(start_state)
            start = dict(zip(state_names, state_positions))
            if any(name not in start or not math.isfinite(start[name]) for name in names):
                raise ValueError('OBSERVATION_BRANCH_START_STATE_INVALID')
            self.manipulator.set_start_state(start_state)
            if hint is None:
                self.manipulator.set_pose_target(pose)
            else:
                goal_by_name = dict(zip(hint['names'], hint['goal']))
                goal = [goal_by_name[name] for name in names]
                seed = deepcopy(start_state)
                seed.joint_state.position = [goal_by_name.get(name, value)
                                            for name, value in zip(state_names, state_positions)]
                self._observation_branch_fk_check(seed, pose, stage='hint_seed')
                configured_joint_tolerance = float(self.manipulator.get_goal_joint_tolerance())
                if not math.isfinite(configured_joint_tolerance) or configured_joint_tolerance < 0.0:
                    raise ValueError('OBSERVATION_BRANCH_JOINT_TOLERANCE_INVALID')
                previous_joint_tolerance = configured_joint_tolerance
                # Joint-goal sampling inside the ordinary +/- tolerance can
                # move an already near-boundary pose outside its original FK
                # tolerance. Request the exact checked q_goal BEFORE collision
                # planning; never repair/overwrite trajectory points afterward.
                self.manipulator.set_goal_joint_tolerance(0.0)
                self.manipulator.set_joint_value_target(goal_by_name)
            result = self.manipulator.plan()
            if not self._plan_success(result):
                raise ValueError('OBSERVATION_BRANCH_PLAN_FAILED' if hint is not None
                                 else 'OBSERVATION_POSE_PLAN_FAILED')
            plan = self._executable_plan(result)
            trajectory = plan.joint_trajectory
            plan_names = tuple(trajectory.joint_names)
            points = list(trajectory.points)
            if (len(plan_names) != 6 or set(plan_names) != set(names) or len(points) < 2
                    or getattr(getattr(plan, 'multi_dof_joint_trajectory', None), 'points', [])
                    or any(len(p.positions) != 6 or not all(math.isfinite(v) for v in p.positions)
                           for p in points)):
                raise ValueError('OBSERVATION_BRANCH_TRAJECTORY_INVALID')
            endpoint = dict(zip(plan_names, points[-1].positions))
            if hint is not None:
                # Only double-precision round-trip noise is admissible here,
                # not the previous physical joint-goal sampling neighborhood.
                if max(abs(endpoint[name] - goal_by_name[name]) for name in names) > 1e-12:
                    raise ValueError('OBSERVATION_BRANCH_JOINT_GOAL_CHANGED')
            terminal_state = self._robot_state_after_plan(start_state, plan)
            if terminal_state is None:
                raise ValueError('OBSERVATION_BRANCH_TERMINAL_STATE_INVALID')
            self._observation_branch_fk_check(terminal_state, pose)
            if (rospy.get_param('/robot_description', '') != description
                    or self._observation_branch_signature()[0] != key[0]):
                raise ValueError('OBSERVATION_BRANCH_MODEL_CHANGED')
            self._remember_pose_plan(pose, plan, 'strict pose')
            if hint is None:
                goal = tuple(float(endpoint[name]) for name in names)
                hint = {'signature': key[0], 'created_sec': now, 'names': names, 'goal': goal,
                        'goal_sha256': hashlib.sha256(json.dumps(
                            [key, names, goal], separators=(',', ':'),
                        ).encode('utf-8')).hexdigest()}
                mode = 'captured'
            else:
                mode = 'fresh_start_fixed_joint_goal'
            cache[key] = hint  # Do not renew the original hint lifetime.
            cache.move_to_end(key)
            while len(cache) > self.OBSERVATION_BRANCH_HINT_CAPACITY:
                cache.popitem(last=False)
            self._last_pose_plan['observation_branch'] = {
                'mode': mode, 'goal_sha256': hint['goal_sha256'],
                'goal_joint_positions': list(hint['goal']),
                'joint_names': list(names),
                'robot_description_sha256': hashlib.sha256(
                    description.encode('utf-8')).hexdigest(),
            }
            return True
        except Exception:
            # Failed revalidation cannot leave a predecessor executable and
            # must not fall through to random IK in this planning invocation.
            self._last_pose_plan = None
            cache.pop(key, None)
            raise
        finally:
            if previous_joint_tolerance is not None:
                try:
                    self.manipulator.set_goal_joint_tolerance(previous_joint_tolerance)
                except Exception as exc:
                    self._last_pose_plan = None
                    cache.pop(key, None)
                    raise ValueError('OBSERVATION_BRANCH_JOINT_TOLERANCE_RESTORE_FAILED: %s' % exc)

    def _attempt_position_target(self, pose, execute):
        if not hasattr(self.manipulator, 'set_position_target'):
            return False
        position = pose.position
        self.manipulator.set_position_target([float(position.x), float(position.y), float(position.z)])
        try:
            return self._run_current_target(execute, pose, 'position-only')
        finally:
            self._clear_targets()

    def _run_current_target(self, execute, pose=None, plan_kind='target'):
        if execute:
            ok = self.manipulator.go(wait=True)
            self.manipulator.stop()
            if ok:
                self._last_pose_plan = None
            return bool(ok)
        # Never retain an older trajectory after a newer plan attempt.  In
        # particular, a failed strict re-check must not leave its predecessor
        # executable.
        self._last_pose_plan = None
        if hasattr(self.manipulator, 'set_start_state_to_current_state'):
            self.manipulator.set_start_state_to_current_state()
        plan = self.manipulator.plan()
        ok = self._plan_success(plan)
        if ok and pose is not None:
            self._remember_pose_plan(pose, self._executable_plan(plan), plan_kind)
        return ok

    def _execute_cached_pose_plan(self, pose, target_text, required_kind=None,
                                 before_execute=None):
        cached = self._matching_cached_pose_plan(pose, required_kind=required_kind)
        if cached is None:
            return False, None
        if cached.get('kind') == 'position-only' and not getattr(self, 'position_only_execute_enabled', False):
            return False, 'execute blocked for position-only cached plan: %s' % target_text
        if not hasattr(self.manipulator, 'execute'):
            return False, 'execute failed from cached plan: MoveIt execute API unavailable; %s' % target_text
        execution_plan = cached['plan']
        retime_message = ''
        if cached.get('kind') == 'strict pose':
            execution_plan, retime_message = self._retime_strict_execution_plan(
                execution_plan
            )
            if execution_plan is None:
                return False, (
                    'strict cached execute blocked: %s; %s'
                    % (retime_message, target_text)
                )
            cached['plan'] = execution_plan
            cached['metrics'] = self._plan_joint_path_metrics(execution_plan)
            retime_message += self._observation_branch_diagnostic(cached)
            if retime_message:
                rospy.loginfo('%s; %s', retime_message, target_text)
        duration_before_reference_sec = self._trajectory_duration_sec(execution_plan)
        execution_plan, reference_evidence, reference_error = (
            self._prepare_controller_reference_timing(execution_plan))
        if reference_error:
            self._last_pose_plan = None
            return False, reference_error
        cached['plan'] = execution_plan
        cached['metrics'] = self._plan_joint_path_metrics(execution_plan)
        guard_error = self._observation_path_guard_error(execution_plan)
        if guard_error:
            self._last_pose_plan = None
            return False, 'strict cached execute blocked: OBSERVATION_PATH_INVALID: ' + guard_error
        reference_error = self._controller_reference_execution_error(
            execution_plan, reference_evidence)
        if reference_error:
            self._last_pose_plan = None
            return False, reference_error
        # The reference bridge can stretch the already-retimed trajectory.
        # Report the trajectory actually submitted, rather than leaving the
        # earlier duration as the apparent execution time in the service result.
        final_duration_sec = self._trajectory_duration_sec(execution_plan)
        submission_timing = 'submitted trajectory duration=%.3fs' % final_duration_sec
        if duration_before_reference_sec > 0.0:
            submission_timing += ' controller_reference_time_scale=%.6f' % (
                final_duration_sec / duration_before_reference_sec)
        retime_message = '; '.join(filter(None, (retime_message, submission_timing)))
        try:
            if before_execute is not None:
                before_execute()
            rospy.loginfo('%s; %s', submission_timing, target_text)
            if getattr(self, 'observation_tracking_contract_required', False):
                ok, contract_message = self._execute_observation_contract(execution_plan, reference_evidence)
                if not ok:
                    # Contract failures cannot become success through an
                    # endpoint-only check after the controller aborted.
                    return False, contract_message
            else:
                ok = self.manipulator.execute(execution_plan, wait=True)
                self.manipulator.stop()
            if ok:
                completion_error = self._controller_reference_completion_error(execution_plan)
                if completion_error:
                    return False, completion_error
                suffix = '; %s' % retime_message if retime_message else ''
                return True, 'executed cached plan (%s): %s%s' % (
                    cached.get('kind', 'target'),
                    target_text,
                    suffix,
                )
            settled, settled_message = self._cached_plan_goal_reached_with_hardware_tolerance(
                execution_plan
            )
            if settled:
                return True, (
                    'executed cached plan (%s): %s; controller reported failure but %s; %s'
                    % (cached.get('kind', 'target'), target_text, settled_message,
                       retime_message)
                )
            return False, (
                'execute failed from cached plan (%s): %s; check trajectory controllers, hardware state, and heat protection'
            ) % (cached.get('kind', 'target'), target_text)
        except Exception as exc:
            return False, 'execute cached plan exception: %s; %s' % (exc, target_text)
        finally:
            # A trajectory is valid only from the state where it was planned.
            # Never replay it after a partial or failed hardware execution.
            self._last_pose_plan = None

    def _execute_observation_contract(self, plan, reference_evidence):
        from alicia_flexible_grasp.robot.contracted_trajectory import bound_action_goal
        audit = getattr(self, '_last_observation_path_audit', {})
        executor = getattr(self, 'observation_trajectory_executor', None)
        authorized = getattr(self, 'observation_execution_authorized', None)
        revalidate = getattr(self, 'observation_submission_revalidate', None)
        if executor is None or not callable(authorized) or not callable(revalidate):
            raise ValueError('bound observation executor or live authority is unavailable')
        goal = bound_action_goal(plan, audit, audit['controller_constraints_snapshot'],
                                 audit['execution_tracking_contract']['stop_trajectory_duration_sec'],
                                 reference=(reference_evidence or {}).get('reference'))
        def before_send():
            revalidate(plan, audit)
            error = self._controller_reference_execution_error(plan, reference_evidence)
            if error:
                raise ValueError(error)
        rospy.loginfo('Submitting proof-bound observation path tolerances: %s',
                      json.dumps(audit['execution_tracking_contract'], sort_keys=True))
        return executor.execute(goal, before_send=before_send, still_authorized=authorized)

    def _controller_reference_completion_error(self, plan):
        """Record successful wait=True execution, never infer it from feedback."""
        callback = getattr(self, 'controller_reference_completion_callback', None)
        if callback is None:
            return ''
        try:
            if not callable(callback):
                raise ValueError('completion provenance callback is not callable')
            callback(deepcopy(plan))
            return ''
        except Exception as exc:
            # This is post-execution, not a pre-submission reference rejection.
            return ('EXECUTION_COMPLETED_PROVENANCE_FAILED: '
                    'execution completed but provenance failed: %s' % exc)

    def _controller_reference_velocity_limits(self, names):
        if len(names) != 6 or set(names) != {'Joint%d' % i for i in range(1, 7)}:
            raise ValueError('controller bridge requires exactly six named arm joints')
        global_limit = float(getattr(
            self, 'strict_execution_max_joint_velocity_rad_s', 0.08))
        configured = getattr(self, 'strict_execution_joint_velocity_limits_rad_s', {})
        if not math.isfinite(global_limit) or global_limit <= 0.0:
            raise ValueError('invalid global joint velocity limit')
        if not isinstance(configured, dict):
            raise ValueError('invalid per-joint velocity limits')
        result = {}
        for name in names:
            limit = float(configured.get(name, global_limit))
            if not math.isfinite(limit) or limit <= 0.0:
                raise ValueError('invalid velocity limit for %s' % name)
            result[name] = min(global_limit, limit)
        return result

    def _controller_reference_snapshot(self, names):
        provider = getattr(self, 'controller_reference_provider', None)
        if not callable(provider):
            raise ValueError('fresh controller reference provider unavailable')
        reference = provider(list(names))
        values = {}
        for field in ('positions', 'velocities', 'accelerations'):
            values[field] = tuple(float(v) for v in getattr(reference, field))
            if len(values[field]) != len(names) or not all(
                    math.isfinite(v) for v in values[field]):
                raise ValueError('controller reference %s must be complete and finite' % field)
        if any(v != 0.0 for field in ('velocities', 'accelerations')
               for v in values[field]):
            raise ValueError('controller reference must be stationary')
        return SimpleNamespace(**values)

    def _prepare_controller_reference_timing(self, plan):
        """Prove the actual Noetic bridge; only uniformly stretch its timing.

        With a stationary external reference, scaling all trajectory times,
        velocities and accelerations preserves every normalized spline curve.
        Neither measured planning start nor any joint position is replaced.
        """
        if not getattr(self, 'controller_reference_guard_required', False):
            return plan, None, ''
        try:
            from alicia_flexible_grasp.robot.observation_path_guard import (
                ObservationVelocityLimitError,
                validate_observation_trajectory_velocity, wire_digest,
            )
            names = tuple(plan.joint_trajectory.joint_names)
            limits = self._controller_reference_velocity_limits(names)
            original_digest = wire_digest(plan)
            original_geometry = deepcopy(plan)
        except Exception as exc:
            return None, None, 'CONTROLLER_BRIDGE_INVALID: %s' % exc
        try:
            reference = self._controller_reference_snapshot(names)
        except Exception as exc:
            return None, None, 'CONTROLLER_REFERENCE_INVALID: %s' % exc
        try:
            if wire_digest(plan) != original_digest:
                raise ValueError('trajectory changed while acquiring controller reference')
            time_scale = 1.0
            execution_plan = plan
            try:
                audit = validate_observation_trajectory_velocity(plan, reference, limits)
            except ObservationVelocityLimitError as exc:
                # Only a completed speed proof is retimable. Missing data,
                # exhausted budgets or any other failure never enter here.
                excess_audit = exc.audit
                if excess_audit.get('trajectory_sha256') != original_digest:
                    raise ValueError('speed proof is not bound to this trajectory')
                required_scale = float(excess_audit['required_time_scale'])
                if not math.isfinite(required_scale) or required_scale <= 1.0:
                    raise ValueError('invalid certified time scale')
                # Time-side margin handles ROS nanosecond rounding; the speed
                # limits themselves are unchanged and proved again below.
                time_scale = required_scale * (1.0 + 1e-6)
                execution_plan, error = self._stretch_trajectory_timing(plan, time_scale)
                if execution_plan is None:
                    raise ValueError('controller bridge time stretch failed: %s' % error)
                if not self._same_joint_path_geometry(
                        original_geometry, execution_plan, tolerance=0.0):
                    raise ValueError('controller bridge time stretch changed joint positions')
                audit = validate_observation_trajectory_velocity(
                    execution_plan, reference, limits)
            if getattr(self, 'observation_tracking_contract_required', False):
                from alicia_flexible_grasp.robot.observation_tracking_contract import (
                    required_command_time_scale, validate_command_derivatives,
                )
                required_scale = required_command_time_scale(execution_plan, reference)
                if required_scale > 1.:
                    contract_scale = required_scale * (1. + 1e-6)
                    execution_plan, error = self._stretch_trajectory_timing(execution_plan, contract_scale)
                    if execution_plan is None:
                        raise ValueError('tracking contract time stretch failed: %s' % error)
                    time_scale *= contract_scale
                    if not self._same_joint_path_geometry(original_geometry, execution_plan, tolerance=0.):
                        raise ValueError('tracking contract time stretch changed joint positions')
                    audit = validate_observation_trajectory_velocity(execution_plan, reference, limits)
                audit['stopping_contract_derivatives'] = validate_command_derivatives(execution_plan, reference)
            if wire_digest(plan) != original_digest:
                raise ValueError('input trajectory changed during speed proof')
            final_digest = wire_digest(execution_plan)
            if not isinstance(audit, dict) or audit.get('trajectory_sha256') != final_digest:
                raise ValueError('final speed proof is not bound to execution trajectory')
            evidence = dict(joint_names=names, reference=reference,
                            limits=limits, trajectory_sha256=final_digest)
            self._last_controller_reference_audit = dict(
                audit, applied_time_scale=time_scale,
                final_duration_sec=self._trajectory_duration_sec(execution_plan))
            rospy.loginfo('Controller reference continuous speed audit: %s',
                          json.dumps(self._last_controller_reference_audit,
                                     sort_keys=True, separators=(',', ':')))
            return execution_plan, evidence, ''
        except Exception as exc:
            return None, None, 'CONTROLLER_BRIDGE_INVALID: %s' % exc

    def _controller_reference_execution_error(self, plan, evidence):
        """Re-read live ownership/freshness after all proof and CAD work."""
        if evidence is None:
            return ('CONTROLLER_REFERENCE_INVALID: no bound controller speed proof'
                    if getattr(self, 'controller_reference_guard_required', False) else '')
        try:
            reference = self._controller_reference_snapshot(evidence['joint_names'])
            if any(getattr(reference, field) != getattr(evidence['reference'], field)
                   for field in ('positions', 'velocities', 'accelerations')):
                raise ValueError('controller reference changed after speed proof')
        except Exception as exc:
            return 'CONTROLLER_REFERENCE_INVALID: %s' % exc
        try:
            from alicia_flexible_grasp.robot.observation_path_guard import wire_digest
            if tuple(plan.joint_trajectory.joint_names) != evidence['joint_names']:
                raise ValueError('execution trajectory joint names changed')
            if self._controller_reference_velocity_limits(evidence['joint_names']) != evidence['limits']:
                raise ValueError('joint velocity limits changed after speed proof')
            if wire_digest(plan) != evidence['trajectory_sha256']:
                raise ValueError('execution trajectory changed after speed proof')
            if getattr(self, 'observation_path_guard_required', False):
                path_audit = getattr(self, '_last_observation_path_audit', {})
                if path_audit.get('trajectory_sha256') != evidence['trajectory_sha256']:
                    raise ValueError('CAD proof is not bound to the final timed trajectory')
            return ''
        except Exception as exc:
            return 'CONTROLLER_BRIDGE_INVALID: %s' % exc

    def _observation_path_guard_error(self, plan):
        if not getattr(self, 'observation_path_guard_required', False):
            return ''
        validator = getattr(self, 'observation_path_validator', None)
        if not callable(validator):
            return 'frozen observation path validator is unavailable'
        try:
            audit = validator(plan)
            if not isinstance(audit, dict) or not audit.get('trajectory_sha256'):
                return 'observation path validator returned no bound evidence'
            self._last_observation_path_audit = audit
            rospy.loginfo('Frozen observation continuous path audit: %s',
                          json.dumps(audit, sort_keys=True, separators=(',', ':')))
            return ''
        except Exception as exc:
            return str(exc)

    def _retime_strict_execution_plan(self, plan):
        if not bool(getattr(self, 'strict_execution_retime_enabled', False)):
            return plan, ''
        if not hasattr(self.manipulator, 'retime_trajectory') or not hasattr(self, 'robot'):
            return None, 'strict execution retiming API is unavailable'
        if not hasattr(self.robot, 'get_current_state'):
            return None, 'strict execution current robot state is unavailable'

        velocity_scaling = min(
            1.0,
            max(
                0.01,
                float(getattr(self, 'strict_execution_velocity_scaling', 0.20)),
            ),
        )
        acceleration_scaling = min(
            1.0,
            max(
                0.01,
                float(
                    getattr(
                        self,
                        'strict_execution_acceleration_scaling',
                        0.30,
                    )
                ),
            ),
        )
        try:
            retimed = self.manipulator.retime_trajectory(
                self.robot.get_current_state(),
                plan,
                velocity_scaling_factor=velocity_scaling,
                acceleration_scaling_factor=acceleration_scaling,
            )
        except Exception as exc:
            return None, 'strict execution retiming failed: %s' % exc
        if not self._plan_success(retimed):
            return None, 'strict execution retiming returned an empty trajectory'
        if not self._same_joint_path_geometry(plan, retimed):
            return None, 'strict execution retiming changed the planned joint path'

        trajectory = getattr(retimed, 'joint_trajectory', None)
        points = list(getattr(trajectory, 'points', []) or [])
        start_positions = list(
            getattr(points[0], 'positions', []) or []
        ) if points else []
        if not start_positions:
            return None, 'strict execution trajectory has no joint start state'
        if not hasattr(self.manipulator, 'get_current_joint_values'):
            return None, 'strict execution live joint feedback is unavailable'
        try:
            current_positions = list(
                self.manipulator.get_current_joint_values() or []
            )
        except Exception as exc:
            return None, 'strict execution live joint feedback failed: %s' % exc
        size = min(len(start_positions), len(current_positions))
        if size <= 0:
            return None, 'strict execution live joint feedback is empty'
        start_errors = [
            abs(float(start_positions[index]) - float(current_positions[index]))
            for index in range(size)
        ]
        if not all(math.isfinite(value) for value in start_errors):
            return None, 'strict execution start-state error is not finite'
        start_error = max(start_errors)
        start_tolerance = max(
            0.0,
            float(
                getattr(
                    self,
                    'strict_execution_start_tolerance_rad',
                    0.08,
                )
            ),
        )
        if start_error > start_tolerance:
            return None, (
                'strict execution trajectory start mismatch %.6frad > %.6frad; '
                'replan from current joint feedback'
                % (start_error, start_tolerance)
            )

        duration_sec = self._trajectory_duration_sec(retimed)
        if not math.isfinite(duration_sec) or duration_sec <= 0.0:
            return None, 'strict execution retiming produced invalid trajectory timing'
        minimum_duration_sec = max(
            0.0,
            float(
                getattr(
                    self,
                    'strict_execution_min_duration_sec',
                    3.0,
                )
            ),
        )
        duration_scale = 1.0
        if duration_sec + 1e-6 < minimum_duration_sec:
            duration_scale = minimum_duration_sec / duration_sec
            retimed, stretch_error = self._stretch_trajectory_timing(
                retimed,
                duration_scale,
            )
            if retimed is None:
                return None, (
                    'strict execution minimum-duration stretch failed: %s'
                    % stretch_error
                )
            if not self._same_joint_path_geometry(plan, retimed):
                return None, (
                    'strict execution minimum-duration stretch changed the '
                    'planned joint path'
                )
            duration_sec = self._trajectory_duration_sec(retimed)
            if (
                not math.isfinite(duration_sec)
                or duration_sec + 1e-6 < minimum_duration_sec
            ):
                return None, (
                    'strict execution minimum-duration stretch produced '
                    'invalid timing %.6fs < %.6fs'
                    % (duration_sec, minimum_duration_sec)
                )
        metrics = self._plan_joint_path_metrics(retimed)
        max_delta = float(metrics.get('max_delta', 0.0))
        max_velocity = max(
            1e-6,
            float(
                getattr(
                    self,
                    'strict_execution_max_joint_velocity_rad_s',
                    0.08,
                )
            ),
        )
        peak_velocities, peak_error = self._trajectory_peak_joint_velocities_rad_s(
            retimed
        )
        if peak_velocities is None:
            return None, 'strict execution velocity validation failed: %s' % peak_error
        joint_names = list(
            getattr(getattr(retimed, 'joint_trajectory', None), 'joint_names', [])
            or []
        )
        configured_joint_limits = dict(
            getattr(
                self,
                'strict_execution_joint_velocity_limits_rad_s',
                {},
            )
            or {}
        )
        velocity_ratios = []
        for index, peak_velocity in enumerate(peak_velocities):
            joint_name = (
                str(joint_names[index])
                if index < len(joint_names)
                else 'joint_%d' % index
            )
            joint_limit = max_velocity
            configured_limit = configured_joint_limits.get(joint_name)
            if configured_limit is not None:
                try:
                    configured_limit = float(configured_limit)
                except (TypeError, ValueError):
                    return None, (
                        'strict execution joint velocity limit for %s is invalid'
                        % joint_name
                    )
                if not math.isfinite(configured_limit) or configured_limit <= 0.0:
                    return None, (
                        'strict execution joint velocity limit for %s is invalid'
                        % joint_name
                    )
                joint_limit = min(joint_limit, configured_limit)
            velocity_ratios.append(
                (float(peak_velocity) / joint_limit, joint_name, joint_limit)
            )
        limiting_ratio, limiting_joint, limiting_velocity = max(
            velocity_ratios or [(0.0, 'none', max_velocity)],
            key=lambda item: item[0],
        )
        velocity_time_scale = max(1.0, limiting_ratio)
        if velocity_time_scale > 1.0 + 1e-9:
            retimed, stretch_error = self._stretch_trajectory_timing(
                retimed,
                velocity_time_scale,
            )
            if retimed is None:
                return None, (
                    'strict execution local-velocity stretch failed: %s'
                    % stretch_error
                )
            if not self._same_joint_path_geometry(plan, retimed):
                return None, (
                    'strict execution local-velocity stretch changed the '
                    'planned joint path'
                )
            duration_scale *= velocity_time_scale
            duration_sec = self._trajectory_duration_sec(retimed)
            peak_velocities, peak_error = (
                self._trajectory_peak_joint_velocities_rad_s(retimed)
            )
            if peak_velocities is None:
                return None, (
                    'strict execution stretched-velocity validation failed: %s'
                    % peak_error
                )
            for index, peak_velocity in enumerate(peak_velocities):
                joint_name = (
                    str(joint_names[index])
                    if index < len(joint_names)
                    else 'joint_%d' % index
                )
                joint_limit = max_velocity
                if joint_name in configured_joint_limits:
                    joint_limit = min(
                        joint_limit,
                        float(configured_joint_limits[joint_name]),
                    )
                if float(peak_velocity) > joint_limit + 1e-6:
                    return None, (
                        'strict execution local velocity %.6frad/s for %s '
                        'exceeds %.6frad/s after time scaling'
                        % (peak_velocity, joint_name, joint_limit)
                    )
        final_peak_velocity = max(peak_velocities or [0.0])
        required_duration = max_delta / max_velocity
        if duration_sec + 1e-6 < required_duration:
            return None, (
                'strict execution trajectory duration %.3fs is below '
                'hardware-following minimum %.3fs'
                % (duration_sec, required_duration)
            )
        return retimed, (
            'strict trajectory retimed duration=%.3fs '
            'max_delta=%.3frad peak_velocity=%.3frad/s '
            'velocity_limit=%.3frad/s limiting_joint=%s '
            'limiting_velocity=%.3frad/s velocity_time_scale=%.3f '
            'start_error=%.6frad minimum_duration=%.3fs time_scale=%.3f'
            % (
                duration_sec,
                max_delta,
                final_peak_velocity,
                max_velocity,
                limiting_joint,
                limiting_velocity,
                velocity_time_scale,
                start_error,
                minimum_duration_sec,
                duration_scale,
            )
        )

    @staticmethod
    def _trajectory_peak_joint_velocities_rad_s(plan):
        trajectory = getattr(plan, 'joint_trajectory', None)
        points = list(getattr(trajectory, 'points', []) or [])
        if not points:
            return None, 'trajectory has no points'
        width = len(list(getattr(points[0], 'positions', []) or []))
        if width <= 0:
            return None, 'trajectory has no joint positions'
        peaks = [0.0] * width
        previous_positions = None
        previous_time_sec = None
        for point in points:
            positions = list(getattr(point, 'positions', []) or [])
            if len(positions) != width:
                return None, 'trajectory joint position widths are inconsistent'
            if not all(math.isfinite(float(value)) for value in positions):
                return None, 'trajectory joint position is not finite'
            timing = getattr(point, 'time_from_start', None)
            if timing is None or not hasattr(timing, 'to_sec'):
                return None, 'trajectory point has no readable time_from_start'
            time_sec = float(timing.to_sec())
            if not math.isfinite(time_sec) or time_sec < 0.0:
                return None, 'trajectory point timing is invalid'
            velocities = list(getattr(point, 'velocities', []) or [])
            if velocities:
                if len(velocities) != width:
                    return None, 'trajectory joint velocity widths are inconsistent'
                if not all(math.isfinite(float(value)) for value in velocities):
                    return None, 'trajectory joint velocity is not finite'
                for index, value in enumerate(velocities):
                    peaks[index] = max(peaks[index], abs(float(value)))
            if previous_positions is not None:
                delta_time_sec = time_sec - previous_time_sec
                if delta_time_sec <= 0.0:
                    return None, 'trajectory point timing is not strictly increasing'
                for index, value in enumerate(positions):
                    finite_difference = abs(
                        float(value) - float(previous_positions[index])
                    ) / delta_time_sec
                    peaks[index] = max(peaks[index], finite_difference)
            previous_positions = positions
            previous_time_sec = time_sec
        return peaks, ''

    @staticmethod
    def _stretch_trajectory_timing(plan, scale):
        if not math.isfinite(float(scale)) or float(scale) < 1.0:
            return None, 'time scale must be finite and at least 1.0'
        stretched = deepcopy(plan)
        trajectory = getattr(stretched, 'joint_trajectory', None)
        points = list(getattr(trajectory, 'points', []) or [])
        if not points:
            return None, 'trajectory has no points'
        for point in points:
            timing = getattr(point, 'time_from_start', None)
            if timing is None or not hasattr(timing, 'to_sec'):
                return None, 'trajectory point has no readable time_from_start'
            old_time_sec = float(timing.to_sec())
            if not math.isfinite(old_time_sec) or old_time_sec < 0.0:
                return None, 'trajectory point timing is invalid'
            point.time_from_start = rospy.Duration.from_sec(
                old_time_sec * float(scale)
            )
            velocities = list(getattr(point, 'velocities', []) or [])
            if velocities:
                point.velocities = [
                    float(value) / float(scale)
                    for value in velocities
                ]
            accelerations = list(getattr(point, 'accelerations', []) or [])
            if accelerations:
                point.accelerations = [
                    float(value) / (float(scale) ** 2)
                    for value in accelerations
                ]
        return stretched, ''

    @staticmethod
    def _same_joint_path_geometry(first, second, tolerance=1e-9):
        first_trajectory = getattr(first, 'joint_trajectory', None)
        second_trajectory = getattr(second, 'joint_trajectory', None)
        first_names = list(getattr(first_trajectory, 'joint_names', []) or [])
        second_names = list(getattr(second_trajectory, 'joint_names', []) or [])
        if first_names != second_names:
            return False
        first_points = list(getattr(first_trajectory, 'points', []) or [])
        second_points = list(getattr(second_trajectory, 'points', []) or [])
        if len(first_points) != len(second_points):
            return False
        for first_point, second_point in zip(first_points, second_points):
            first_positions = list(getattr(first_point, 'positions', []) or [])
            second_positions = list(getattr(second_point, 'positions', []) or [])
            if len(first_positions) != len(second_positions):
                return False
            for first_value, second_value in zip(first_positions, second_positions):
                if (
                    not math.isfinite(float(first_value))
                    or not math.isfinite(float(second_value))
                    or abs(float(first_value) - float(second_value)) > tolerance
                ):
                    return False
        return True

    @staticmethod
    def _trajectory_duration_sec(plan):
        trajectory = getattr(plan, 'joint_trajectory', None)
        points = list(getattr(trajectory, 'points', []) or [])
        if not points:
            return 0.0
        duration = getattr(points[-1], 'time_from_start', None)
        if duration is None:
            return 0.0
        if hasattr(duration, 'to_sec'):
            return float(duration.to_sec())
        return float(getattr(duration, 'secs', 0.0)) + (
            float(getattr(duration, 'nsecs', 0.0)) * 1e-9
        )

    def _remember_pose_plan(self, pose, plan, plan_kind):
        xyz = self._pose_xyz_tuple(pose)
        if xyz is None:
            self._last_pose_plan = None
            return
        self._last_pose_plan = {
            'xyz': xyz,
            'quaternion': self._pose_quaternion_tuple(pose),
            'plan': plan,
            'kind': plan_kind,
            'metrics': self._plan_joint_path_metrics(plan),
        }

    def _matching_cached_pose_plan(self, pose, required_kind=None):
        cached = getattr(self, '_last_pose_plan', None)
        if not cached:
            return None
        if required_kind is not None and cached.get('kind') != required_kind:
            return None
        xyz = self._pose_xyz_tuple(pose)
        if xyz is None:
            return None
        cached_xyz = cached.get('xyz')
        if cached_xyz is None:
            return None
        tolerance = max(0.0, float(getattr(self, 'cached_plan_position_tolerance_m', 0.002)))
        deltas = [abs(float(a) - float(b)) for a, b in zip(xyz, cached_xyz)]
        if not all(delta <= tolerance for delta in deltas):
            return None
        cached_quaternion = cached.get('quaternion')
        target_quaternion = self._pose_quaternion_tuple(pose)
        if (
            cached.get('kind') in ('strict pose', 'cartesian')
            and cached_quaternion is not None
            and target_quaternion is not None
        ):
            angle = self._quaternion_angle(cached_quaternion, target_quaternion)
            orientation_tolerance = max(
                0.0,
                float(getattr(self, 'cached_plan_orientation_tolerance_rad', 0.02)),
            )
            if angle > orientation_tolerance:
                return None
        return cached

    def _matching_cached_strict_pose_plan(self, pose):
        cached = getattr(self, '_last_pose_plan', None)
        if not cached:
            return None, 'no cached pose plan'

        kind = cached.get('kind')
        if kind != 'strict pose':
            return None, "cached plan kind %r is not 'strict pose'" % kind

        plan = cached.get('plan')
        if plan is None or not self._plan_success(plan):
            return None, 'cached strict pose trajectory is empty'

        target_xyz = self._pose_xyz_tuple(pose)
        if target_xyz is None:
            return None, 'target position is invalid'
        cached_xyz = cached.get('xyz')
        if not self._finite_tuple(cached_xyz, 3):
            return None, 'cached strict pose position is invalid'

        position_tolerance = max(
            0.0,
            float(getattr(self, 'cached_plan_position_tolerance_m', 0.002)),
        )
        position_delta = max(
            abs(float(target) - float(planned))
            for target, planned in zip(target_xyz, cached_xyz)
        )
        if position_delta > position_tolerance:
            return None, (
                'cached strict pose position mismatch %.6fm > %.6fm'
                % (position_delta, position_tolerance)
            )

        target_quaternion = self._pose_quaternion_tuple(pose)
        if target_quaternion is None:
            return None, 'target orientation is invalid'
        cached_quaternion = cached.get('quaternion')
        if not self._finite_tuple(cached_quaternion, 4):
            return None, 'cached strict pose orientation is invalid'
        cached_norm = math.sqrt(sum(float(value) ** 2 for value in cached_quaternion))
        if cached_norm <= 1e-9:
            return None, 'cached strict pose orientation is invalid'
        cached_quaternion = tuple(float(value) / cached_norm for value in cached_quaternion)
        orientation_delta = self._quaternion_angle(cached_quaternion, target_quaternion)
        orientation_tolerance = max(
            0.0,
            float(getattr(self, 'cached_plan_orientation_tolerance_rad', 0.02)),
        )
        if orientation_delta > orientation_tolerance:
            return None, (
                'cached strict pose orientation mismatch %.6frad > %.6frad'
                % (orientation_delta, orientation_tolerance)
            )
        return cached, None

    def _cached_plan_metrics_text(self):
        cached = getattr(self, '_last_pose_plan', None) or {}
        metrics = cached.get('metrics') or {}
        if not metrics:
            return ''
        message = ' joint_path_cost=%.3f joint_max_delta=%.3f' % (
            float(metrics.get('path_cost', 0.0)),
            float(metrics.get('max_delta', 0.0)),
        )
        duration_lower_bound = float(
            metrics.get('execution_duration_lower_bound_sec', 0.0)
        )
        limiting_joint = str(
            metrics.get('hardware_limiting_joint', '') or ''
        )
        if duration_lower_bound > 0.0 and limiting_joint:
            message += (
                ' joint_duration_lower_bound_sec=%.3f'
                ' execution_duration_lower_bound_sec=%.3f'
                ' hardware_limiting_joint=%s'
            ) % (
                duration_lower_bound,
                duration_lower_bound,
                limiting_joint,
            )
        return message + self._observation_branch_diagnostic(cached)

    @staticmethod
    def _observation_branch_diagnostic(cached):
        branch = cached.get('observation_branch')
        if not branch:
            return ''
        return (' observation_branch=%s observation_goal_sha256=%s observation_goal_joint_positions=%s'
                ' observation_joint_names=%s observation_robot_description_sha256=%s') % (
            branch['mode'], branch['goal_sha256'],
            json.dumps(branch['goal_joint_positions'], separators=(',', ':')),
            json.dumps(branch.get('joint_names', []), separators=(',', ':')),
            branch.get('robot_description_sha256', ''))

    def _plan_joint_path_metrics(self, plan):
        trajectory = getattr(plan, 'joint_trajectory', None)
        points = list(getattr(trajectory, 'points', []) or [])
        positions = [list(getattr(point, 'positions', []) or []) for point in points]
        # Legacy callers may provide trajectory points without positions (for
        # example, a lightweight success sentinel).  Ignore those here; strict
        # sequence validation below rejects incomplete trajectories fail-closed.
        positions = [values for values in positions if values]
        if any(
            not all(math.isfinite(float(value)) for value in values)
            for values in positions
        ):
            raise ValueError('joint trajectory contains non-finite positions')
        if len(positions) < 2:
            return {
                'path_cost': 0.0,
                'max_delta': 0.0,
                'execution_duration_lower_bound_sec': 0.0,
                'joint_duration_lower_bound_sec': 0.0,
                'hardware_limiting_joint': '',
            }
        path_cost = 0.0
        joint_count = min(len(values) for values in positions)
        joint_travel = [0.0] * joint_count
        for previous, current in zip(positions, positions[1:]):
            deltas = [
                float(current[index]) - float(previous[index])
                for index in range(joint_count)
            ]
            path_cost += math.sqrt(sum(delta * delta for delta in deltas))
            for index, delta in enumerate(deltas):
                joint_travel[index] += abs(delta)
        max_delta = max(
            abs(float(positions[-1][index]) - float(positions[0][index]))
            for index in range(joint_count)
        )

        joint_names = list(getattr(trajectory, 'joint_names', []) or [])
        global_limit = float(
            getattr(self, 'strict_execution_max_joint_velocity_rad_s', 0.08)
        )
        if not math.isfinite(global_limit) or global_limit <= 0.0:
            raise ValueError('joint velocity limit is non-finite or non-positive')
        global_limit = max(1e-6, global_limit)
        configured_limits = dict(
            getattr(
                self,
                'strict_execution_joint_velocity_limits_rad_s',
                {},
            )
            or {}
        )
        duration_candidates = []
        for index, travel in enumerate(joint_travel):
            joint_name = (
                str(joint_names[index])
                if index < len(joint_names)
                else 'joint_%d' % index
            )
            limit = global_limit
            if joint_name in configured_limits:
                configured_limit = float(configured_limits[joint_name])
                if not math.isfinite(configured_limit) or configured_limit <= 0.0:
                    raise ValueError(
                        'joint velocity limit for %s is non-finite or non-positive'
                        % joint_name
                    )
                limit = min(limit, configured_limit)
            duration_candidates.append((float(travel) / limit, joint_name))
        duration_lower_bound, limiting_joint = max(
            duration_candidates or [(0.0, '')],
            key=lambda item: item[0],
        )
        if not math.isfinite(path_cost) or not math.isfinite(max_delta):
            raise ValueError('joint trajectory metrics are non-finite')
        if not math.isfinite(duration_lower_bound) or duration_lower_bound < 0.0:
            raise ValueError('joint trajectory duration is non-finite')
        return {
            'path_cost': float(path_cost),
            'max_delta': float(max_delta),
            'execution_duration_lower_bound_sec': float(
                duration_lower_bound
            ),
            'joint_duration_lower_bound_sec': float(duration_lower_bound),
            'hardware_limiting_joint': str(limiting_joint),
        }

    def _cached_plan_goal_reached_with_hardware_tolerance(self, plan):
        tolerance = max(
            0.0,
            float(getattr(self, 'execution_goal_tolerance_rad', 0.03)),
        )
        slack = max(
            0.0,
            float(getattr(self, 'execution_goal_tolerance_slack_rad', 0.005)),
        )
        effective_tolerance = tolerance + slack
        if effective_tolerance <= 0.0:
            return False, ''
        trajectory = getattr(plan, 'joint_trajectory', None)
        points = list(getattr(trajectory, 'points', []) or [])
        if not points:
            return False, ''
        goal_positions = list(getattr(points[-1], 'positions', []) or [])
        if not goal_positions:
            return False, ''
        if not hasattr(self.manipulator, 'get_current_joint_values'):
            return False, ''
        try:
            current_positions = list(self.manipulator.get_current_joint_values() or [])
        except Exception:
            return False, ''
        size = min(len(goal_positions), len(current_positions))
        if size <= 0:
            return False, ''
        errors = [
            abs(float(current_positions[index]) - float(goal_positions[index]))
            for index in range(size)
        ]
        if not all(math.isfinite(value) for value in errors):
            return False, ''
        max_error = max(errors)
        if max_error <= tolerance:
            return True, (
                'joint feedback is within hardware goal tolerance '
                'max_error=%.6frad <= %.6frad'
                % (max_error, tolerance)
            )
        if max_error <= effective_tolerance:
            return True, (
                'joint feedback is within hardware goal tolerance including slack '
                'max_error=%.6frad <= %.6frad (base %.6frad + slack %.6frad)'
                % (max_error, effective_tolerance, tolerance, slack)
            )
        return False, (
            'joint feedback max_error=%.6frad > %.6frad'
            % (max_error, effective_tolerance)
        )

    @staticmethod
    def _pose_position_distance(first, second):
        a = first.position
        b = second.position
        return math.sqrt(
            (float(a.x) - float(b.x)) ** 2
            + (float(a.y) - float(b.y)) ** 2
            + (float(a.z) - float(b.z)) ** 2
        )

    @staticmethod
    def _pose_quaternion_tuple(pose):
        try:
            q = pose.orientation
            values = [float(q.x), float(q.y), float(q.z), float(q.w)]
            if not all(math.isfinite(value) for value in values):
                return None
            norm = math.sqrt(sum(value * value for value in values))
            if norm <= 1e-9:
                return None
            return tuple(value / norm for value in values)
        except Exception:
            return None

    @staticmethod
    def _quaternion_angle(first, second):
        dot = abs(sum(float(a) * float(b) for a, b in zip(first, second)))
        return 2.0 * math.acos(min(1.0, max(-1.0, dot)))

    def _clear_targets(self):
        try:
            self.manipulator.clear_pose_targets()
        except Exception:
            pass

    def _pose_fallbacks_enabled(self):
        return bool(getattr(self, 'pose_fallback_enabled', True))

    def _position_only_fallback_allowed(self, execute):
        if not self._pose_fallbacks_enabled():
            return False
        if not getattr(self, 'position_only_fallback_enabled', True):
            return False
        if execute and not getattr(self, 'position_only_execute_enabled', False):
            return False
        return True

    def _candidate_orientations(self, original_pose):
        seen = set()
        original_key = self._orientation_key(getattr(original_pose, 'orientation', None))
        if original_key is not None:
            seen.add(original_key)

        current = self._current_orientation()
        current_key = self._orientation_key(current)
        if current is not None and current_key not in seen:
            seen.add(current_key)
            yield 'current', current

        for index, values in enumerate(getattr(self, 'candidate_orientations', self.DEFAULT_CANDIDATE_ORIENTATIONS_XYZW), start=1):
            orientation = self._orientation_from_xyzw(values)
            key = self._orientation_key(orientation)
            if key is None or key in seen:
                continue
            seen.add(key)
            yield 'configured#%d' % index, orientation

    def _current_orientation(self):
        try:
            pose_stamped = self.manipulator.get_current_pose()
            pose = getattr(pose_stamped, 'pose', pose_stamped)
            return getattr(pose, 'orientation', None)
        except Exception:
            return None

    def _pose_with_orientation(self, pose, orientation):
        candidate = deepcopy(pose)
        candidate.orientation = SimpleNamespace(
            x=float(orientation.x),
            y=float(orientation.y),
            z=float(orientation.z),
            w=float(orientation.w),
        )
        return candidate

    @staticmethod
    def _orientation_from_xyzw(values):
        if hasattr(values, 'x'):
            return values
        items = list(values)
        if len(items) != 4:
            return None
        return SimpleNamespace(x=float(items[0]), y=float(items[1]), z=float(items[2]), w=float(items[3]))

    @staticmethod
    def _orientation_key(orientation):
        if orientation is None:
            return None
        try:
            return (
                round(float(orientation.x), 4),
                round(float(orientation.y), 4),
                round(float(orientation.z), 4),
                round(float(orientation.w), 4),
            )
        except Exception:
            return None

    @staticmethod
    def _bool_param(private_name, global_name, default):
        try:
            return bool(rospy.get_param(private_name, rospy.get_param(global_name, default)))
        except Exception:
            return bool(default)

    @staticmethod
    def _float_param(private_name, global_name, default):
        try:
            return float(rospy.get_param(private_name, rospy.get_param(global_name, default)))
        except Exception:
            return float(default)

    @staticmethod
    def _joint_velocity_limits_param(private_name, global_name, default):
        try:
            values = rospy.get_param(
                private_name,
                rospy.get_param(global_name, default),
            )
        except Exception:
            values = default
        if not isinstance(values, dict):
            return dict(default)
        limits = {}
        for name, value in values.items():
            try:
                numeric = float(value)
            except (TypeError, ValueError):
                continue
            if math.isfinite(numeric) and numeric > 0.0:
                limits[str(name)] = numeric
        return limits

    @staticmethod
    def _orientation_param(private_name, global_name, default):
        try:
            values = rospy.get_param(private_name, rospy.get_param(global_name, default))
        except Exception:
            values = default
        orientations = []
        for item in values:
            try:
                if len(item) == 4:
                    orientations.append([float(v) for v in item])
            except Exception:
                continue
        return orientations or [list(v) for v in default]

    def _plan_success(self, plan_result):
        if isinstance(plan_result, tuple):
            if len(plan_result) >= 2 and isinstance(plan_result[0], bool):
                return bool(plan_result[0])
            plan_result = plan_result[0] if plan_result else None
        traj = getattr(plan_result, 'joint_trajectory', None)
        return bool(traj and traj.points)

    def _executable_plan(self, plan_result):
        if isinstance(plan_result, tuple):
            if len(plan_result) >= 2 and isinstance(plan_result[0], bool):
                return plan_result[1] if plan_result[0] else None
            return plan_result[0] if plan_result else None
        return plan_result

    @staticmethod
    def _pose_xyz_text(pose):
        try:
            p = pose.position
            return 'target xyz=(%.3f, %.3f, %.3f)' % (float(p.x), float(p.y), float(p.z))
        except Exception:
            return 'target xyz=(unavailable)'

    @staticmethod
    def _pose_xyz_tuple(pose):
        try:
            p = pose.position
            values = (float(p.x), float(p.y), float(p.z))
            return values if all(math.isfinite(value) for value in values) else None
        except Exception:
            return None

    @staticmethod
    def _finite_tuple(values, expected_size):
        try:
            items = tuple(float(value) for value in values)
        except (TypeError, ValueError):
            return False
        return len(items) == int(expected_size) and all(math.isfinite(value) for value in items)
