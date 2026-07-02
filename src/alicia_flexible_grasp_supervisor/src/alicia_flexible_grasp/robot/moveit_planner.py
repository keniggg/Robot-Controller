import sys
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
            0.01,
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
            planning_attempts = int(rospy.get_param('~planning_attempts', rospy.get_param('/robot/planning_attempts', 1)))
            self.manipulator.set_planning_time(max(0.5, planning_time))
            self.manipulator.set_num_planning_attempts(max(1, planning_attempts))
            self.ready = True
        except Exception as exc:
            self.error = str(exc)
            rospy.logwarn('MoveItPlanner not ready: %s', self.error)

    def move_to_pose(self, pose_stamped_or_pose, execute=True):
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
                plan_ok, plan_message = self.move_to_pose(pose, execute=False)
                if not plan_ok:
                    return False, 'execute planning failed before motion: %s' % plan_message
                cached_ok, cached_message = self._execute_cached_pose_plan(pose, target_text)
                if cached_message:
                    return cached_ok, cached_message
                return False, 'execute failed: no executable cached plan after planning; %s' % target_text

            if self._attempt_pose_target(pose, execute):
                return True, '%s: %s' % (action_done, target_text)
            failed_attempts.append('strict pose')

            if self._pose_fallbacks_enabled() and getattr(self, 'orientation_fallback_enabled', True):
                labels = []
                for label, orientation in self._candidate_orientations(pose):
                    labels.append(label)
                    candidate = self._pose_with_orientation(pose, orientation)
                    if self._attempt_pose_target(candidate, execute):
                        return True, '%s with candidate orientation %s: %s' % (action_done, label, target_text)
                if labels:
                    failed_attempts.append('candidate orientations %s' % ','.join(labels))

            if self._position_only_fallback_allowed(execute):
                if self._attempt_position_target(pose, execute):
                    return True, '%s with position-only fallback: %s' % (action_done, target_text)
                failed_attempts.append('position-only')
            elif execute and getattr(self, 'position_only_fallback_enabled', True):
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

    def move_to_joints(self, joints, execute=True):
        if not self.ready:
            return False, self.error or 'MoveIt not ready'
        try:
            self.manipulator.set_joint_value_target(list(joints[:6]))
            if execute:
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

    def _attempt_pose_target(self, pose, execute):
        self.manipulator.set_pose_target(pose)
        try:
            return self._run_current_target(execute, pose, 'strict pose')
        finally:
            self._clear_targets()

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
        plan = self.manipulator.plan()
        ok = self._plan_success(plan)
        if ok and pose is not None:
            self._remember_pose_plan(pose, self._executable_plan(plan), plan_kind)
        return ok

    def _execute_cached_pose_plan(self, pose, target_text):
        cached = self._matching_cached_pose_plan(pose)
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
                self._last_pose_plan = None
                return True, 'executed cached plan (%s): %s' % (cached.get('kind', 'target'), target_text)
            return False, (
                'execute failed from cached plan (%s): %s; check trajectory controllers, hardware state, and heat protection'
            ) % (cached.get('kind', 'target'), target_text)
        except Exception as exc:
            return False, 'execute cached plan exception: %s; %s' % (exc, target_text)

    def _remember_pose_plan(self, pose, plan, plan_kind):
        xyz = self._pose_xyz_tuple(pose)
        if xyz is None:
            self._last_pose_plan = None
            return
        self._last_pose_plan = {'xyz': xyz, 'plan': plan, 'kind': plan_kind}

    def _matching_cached_pose_plan(self, pose):
        cached = getattr(self, '_last_pose_plan', None)
        if not cached:
            return None
        xyz = self._pose_xyz_tuple(pose)
        if xyz is None:
            return None
        cached_xyz = cached.get('xyz')
        if cached_xyz is None:
            return None
        tolerance = max(0.0, float(getattr(self, 'cached_plan_position_tolerance_m', 0.01)))
        deltas = [abs(float(a) - float(b)) for a, b in zip(xyz, cached_xyz)]
        if all(delta <= tolerance for delta in deltas):
            return cached
        return None

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
            return (float(p.x), float(p.y), float(p.z))
        except Exception:
            return None
