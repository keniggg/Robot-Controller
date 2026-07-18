import sys
import math
import inspect
from copy import deepcopy
from types import SimpleNamespace

import rospy

class MoveItPlanner:
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

    def _attempt_pose_target(self, pose, execute, planning_time=None, plan_kind='strict pose'):
        previous_planning_time = None
        if planning_time is not None and hasattr(self.manipulator, 'set_planning_time'):
            if hasattr(self.manipulator, 'get_planning_time'):
                previous_planning_time = self.manipulator.get_planning_time()
            self.manipulator.set_planning_time(max(0.05, float(planning_time)))
        self.manipulator.set_pose_target(pose)
        try:
            return self._run_current_target(execute, pose, plan_kind)
        finally:
            self._clear_targets()
            if previous_planning_time is not None:
                self.manipulator.set_planning_time(previous_planning_time)

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
        plan = self.manipulator.plan()
        ok = self._plan_success(plan)
        if ok and pose is not None:
            self._remember_pose_plan(pose, self._executable_plan(plan), plan_kind)
        return ok

    def _execute_cached_pose_plan(self, pose, target_text, required_kind=None):
        cached = self._matching_cached_pose_plan(pose, required_kind=required_kind)
        if cached is None:
            return False, None
        if cached.get('kind') == 'position-only' and not getattr(self, 'position_only_execute_enabled', False):
            return False, 'execute blocked for position-only cached plan: %s' % target_text
        if not hasattr(self.manipulator, 'execute'):
            return False, 'execute failed from cached plan: MoveIt execute API unavailable; %s' % target_text
        try:
            ok = self.manipulator.execute(cached['plan'], wait=True)
            self.manipulator.stop()
            if ok:
                return True, 'executed cached plan (%s): %s' % (cached.get('kind', 'target'), target_text)
            return False, (
                'execute failed from cached plan (%s): %s; check trajectory controllers, hardware state, and heat protection'
            ) % (cached.get('kind', 'target'), target_text)
        except Exception as exc:
            return False, 'execute cached plan exception: %s; %s' % (exc, target_text)
        finally:
            # A trajectory is valid only from the state where it was planned.
            # Never replay it after a partial or failed hardware execution.
            self._last_pose_plan = None

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
        return ' joint_path_cost=%.3f joint_max_delta=%.3f' % (
            float(metrics.get('path_cost', 0.0)),
            float(metrics.get('max_delta', 0.0)),
        )

    @staticmethod
    def _plan_joint_path_metrics(plan):
        trajectory = getattr(plan, 'joint_trajectory', None)
        points = list(getattr(trajectory, 'points', []) or [])
        positions = [list(getattr(point, 'positions', []) or []) for point in points]
        positions = [values for values in positions if values]
        if len(positions) < 2:
            return {'path_cost': 0.0, 'max_delta': 0.0}
        path_cost = 0.0
        for previous, current in zip(positions, positions[1:]):
            size = min(len(previous), len(current))
            path_cost += math.sqrt(sum((float(current[i]) - float(previous[i])) ** 2 for i in range(size)))
        size = min(len(positions[0]), len(positions[-1]))
        max_delta = max(abs(float(positions[-1][i]) - float(positions[0][i])) for i in range(size))
        return {'path_cost': float(path_cost), 'max_delta': float(max_delta)}

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
