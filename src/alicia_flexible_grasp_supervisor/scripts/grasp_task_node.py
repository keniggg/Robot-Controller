#!/usr/bin/env python3
from copy import deepcopy
import math
import time
import rospy
from geometry_msgs.msg import PoseArray, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from alicia_flexible_grasp_supervisor.msg import ObjectPose, GraspState
from alicia_flexible_grasp_supervisor.srv import StartGrasp, StartGraspResponse, StopGrasp, StopGraspResponse, SetTargetPose, SetFloat
from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages, STATE_NAMES
from alicia_flexible_grasp.grasp.grasp_pose_generator import make_pregrasp_pose, make_lift_pose
from alicia_flexible_grasp.robot.planning_feedback import (
    is_orientation_fallback_message,
    is_position_only_fallback_message,
    orientation_fallback_rejection_message,
    position_only_rejection_message,
)
from alicia_flexible_grasp.vision.mujoco_digital_twin_client import (
    MujocoDigitalTwinClient,
    build_simulation_payload,
)
try:
    import tf2_ros
except Exception:
    tf2_ros = None

class GraspTaskNode:
    def __init__(self):
        self.latest_obj = None
        self.latest_obj_time = None
        self.latest_visual_obj = None
        self.latest_visual_obj_time = None
        self.latest_grasp6d_plan = None
        self.latest_grasp6d_plan_time = None
        self.latest_grasp6d_plan_object = None
        self.latest_joint_state = None
        self.latest_raw_detection = False
        self.latest_raw_detection_time = None
        self.active = False
        self.stage = GraspStages.IDLE
        self.tf_buffer = None
        self.tf_listener = None
        if tf2_ros is not None and bool(rospy.get_param('/handeye/use_tf', True)):
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pub = rospy.Publisher('/grasp/state', GraspState, queue_size=10)
        rospy.Subscriber('/perception/object', ObjectPose, self.obj_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param('/grasp/grasp6d_plan_topic', '/grasp_6d/plan'), PoseArray, self.grasp6d_plan_cb, queue_size=1)
        rospy.Subscriber('/joint_states', JointState, self.joint_cb, queue_size=1)
        rospy.Subscriber('/perception/raw_object_detected', Bool, self.raw_detection_cb, queue_size=1)
        rospy.Service('/grasp/start', StartGrasp, self.start_cb)
        rospy.Service('/grasp/stop', StopGrasp, self.stop_cb)
        rospy.loginfo('GraspTaskNode ready')

    def obj_cb(self, msg):
        if not msg.detected:
            return
        gcfg = rospy.get_param('/grasp', {})
        confidence = float(getattr(msg, 'confidence', 1.0) or 0.0)
        now = rospy.Time.now()
        # Preserve low-confidence close-range detections for visual retargeting.
        # Candidate generation still uses latest_obj and its stricter threshold.
        self.latest_visual_obj = msg
        self.latest_visual_obj_time = now
        min_confidence = self._cfg_float(gcfg, 'min_object_confidence', 0.50)
        if confidence < min_confidence:
            rospy.logwarn_throttle(
                1.0,
                'Grasp ignored low-confidence object %.3f < %.3f',
                confidence,
                min_confidence,
            )
            return

        previous = getattr(self, 'latest_obj', None)
        previous_time = getattr(self, 'latest_obj_time', None)
        max_jump = self._cfg_float(gcfg, 'max_object_jump_m', 0.12)
        jump_window = self._cfg_float(gcfg, 'object_jump_filter_window_sec', 4.0)
        if previous is not None and previous_time is not None and max_jump > 0.0:
            try:
                age = (now - previous_time).to_sec()
            except Exception:
                age = float('inf')
            jump = self._object_distance(previous, msg)
            if age <= jump_window and jump > max_jump:
                rospy.logwarn_throttle(
                    1.0,
                    'Grasp ignored object jump %.3f m > %.3f m within %.1fs',
                    jump,
                    max_jump,
                    jump_window,
                )
                return

        self.latest_obj = msg
        self.latest_obj_time = now

    def joint_cb(self, msg):
        self.latest_joint_state = msg

    def raw_detection_cb(self, msg):
        self.latest_raw_detection = bool(msg.data)
        self.latest_raw_detection_time = rospy.Time.now()

    def grasp6d_plan_cb(self, msg):
        self.latest_grasp6d_plan = msg
        self.latest_grasp6d_plan_time = rospy.Time.now()
        obj = getattr(self, 'latest_obj', None)
        self.latest_grasp6d_plan_object = deepcopy(obj) if obj is not None and bool(getattr(obj, 'detected', False)) else None

    def set_state(self, stage, message='', success=False):
        self.stage = stage
        msg = GraspState()
        msg.header.stamp = rospy.Time.now()
        msg.stage = int(stage)
        msg.state = STATE_NAMES.get(stage, 'UNKNOWN')
        msg.active = self.active
        msg.success = success
        msg.message = message
        self.pub.publish(msg)
        rospy.loginfo('[Grasp] %s %s', msg.state, message)

    def start_cb(self, req):
        if self.active:
            return StartGraspResponse(False, 'already active')
        self.active = True
        try:
            result = self.execute()
            self.active = False
            return StartGraspResponse(result, 'success' if result else 'failed')
        except Exception as exc:
            self.active = False
            self.set_state(GraspStages.FAILED, str(exc), False)
            return StartGraspResponse(False, str(exc))

    def stop_cb(self, req):
        self.active = False
        self.set_state(GraspStages.EMERGENCY_STOP if req.emergency else GraspStages.IDLE, 'stop requested')
        return StopGraspResponse(True, 'stop requested')

    def execute(self):
        rospy.wait_for_service('/supervisor/move_to_pose', timeout=10)
        rospy.wait_for_service('/supervisor/move_to_pose_linear', timeout=10)
        rospy.wait_for_service('/supervisor/set_gripper', timeout=10)
        move_pose = rospy.ServiceProxy('/supervisor/move_to_pose', SetTargetPose)
        move_pose_linear = rospy.ServiceProxy('/supervisor/move_to_pose_linear', SetTargetPose)
        set_gripper = rospy.ServiceProxy('/supervisor/set_gripper', SetFloat)
        gcfg = rospy.get_param('/grasp', {})
        gripper_cfg = rospy.get_param('/gripper', {})
        close = None
        if bool(gripper_cfg.get('use_compliant_close', True)):
            rospy.wait_for_service('/supervisor/compliant_close', timeout=10)
            close = rospy.ServiceProxy('/supervisor/compliant_close', StartGrasp)
        pregrasp_distance = float(gcfg.get('pregrasp_distance', gcfg.get('pregrasp_distance_m', 0.08)))
        final_offset = max(0.0, float(gcfg.get('final_approach_offset_m', 0.015)))
        pregrasp_mode = str(gcfg.get('pregrasp_offset_mode', 'base_z'))
        lift_height = float(gcfg.get('lift_height_m', 0.05))
        pregrasp_reached_tolerance = float(gcfg.get('pregrasp_reached_tolerance_m', 0.04))
        open_position = float(gripper_cfg.get('open_position_m', 0.0))

        if bool(gcfg.get('use_grasp6d_plan', False)):
            return self._execute_grasp6d_plan(
                gcfg,
                gripper_cfg,
                open_position,
                move_pose,
                move_pose_linear,
                set_gripper,
                close,
            )

        self.set_state(GraspStages.SEARCH_OBJECT, 'waiting for object')
        t0 = rospy.Time.now()
        while self.latest_obj is None and (rospy.Time.now()-t0).to_sec() < 5.0 and self.active:
            rospy.sleep(0.05)
        if self.latest_obj is None:
            self.set_state(GraspStages.FAILED, 'no object')
            return False
        locked_obj = deepcopy(self.latest_obj)
        self._log_object_pose('locked target', locked_obj)

        self.set_state(GraspStages.PLAN_PREGRASP, 'compute pregrasp')
        camera_pose = self._lookup_camera_pose_base()
        pre = make_pregrasp_pose(
            locked_obj.pose_base,
            pregrasp_distance,
            camera_pose=camera_pose,
            mode=pregrasp_mode,
        )
        if self._pose_close_enough(self._current_tool_pose_base(), pre, pregrasp_reached_tolerance):
            self.set_state(GraspStages.MOVE_PREGRASP, 'already at pregrasp')
        else:
            self.set_state(GraspStages.MOVE_PREGRASP, 'planning')
            resp = move_pose(pre, False)
            if not resp.success:
                self.set_state(GraspStages.FAILED, 'pregrasp planning failed: ' + resp.message)
                return False
            self.set_state(GraspStages.MOVE_PREGRASP, 'moving')
            resp = move_pose(pre, True)
            if not resp.success:
                self.set_state(GraspStages.FAILED, resp.message)
                return False
            self._wait_for_motion_settle('pregrasp')

        if not self._command_gripper_position(
            set_gripper,
            open_position,
            'open gripper',
            self._cfg_float(gripper_cfg, 'open_wait_sec', 0.5),
        ):
            return False
        self._wait_for_motion_settle('before approach')

        if not self.active:
            self.set_state(GraspStages.IDLE, 'stopped before target approach')
            return False

        camera_pose = self._lookup_camera_pose_base() or camera_pose
        target_obj = self._target_for_approach(locked_obj, gcfg)
        self._log_object_pose('approach target', target_obj)
        approach = make_pregrasp_pose(
            target_obj.pose_base,
            final_offset,
            camera_pose=camera_pose,
            mode=pregrasp_mode,
        )
        self.set_state(GraspStages.APPROACH_TARGET, 'planning target approach')
        resp = move_pose_linear(approach, False)
        if not resp.success:
            self.set_state(GraspStages.FAILED, 'approach target planning failed: ' + resp.message)
            return False

        self.set_state(GraspStages.APPROACH_TARGET, 'moving to target')
        resp = move_pose_linear(approach, True)
        if not resp.success:
            self.set_state(GraspStages.FAILED, 'approach target failed: ' + resp.message)
            return False
        self._wait_for_motion_settle('approach')

        close_label = 'force-guided close' if bool(gripper_cfg.get('use_compliant_close', True)) else 'fixed gripper close'
        self.set_state(GraspStages.COMPLIANT_CLOSE, close_label)
        ok, message = self._close_gripper(gripper_cfg, set_gripper, close)
        if not ok:
            self.set_state(GraspStages.FAILED, message)
            return False

        self.set_state(GraspStages.LIFT_OBJECT, 'lifting')
        lift = make_lift_pose(approach, lift_height)
        if not self._plan_and_execute_pose(
            GraspStages.LIFT_OBJECT,
            'linear lift',
            lift,
            move_pose_linear,
            'lift',
        ):
            return False
        self.set_state(GraspStages.SUCCESS, 'grasp done', True)
        return True

    def _execute_grasp6d_plan(
        self,
        gcfg,
        gripper_cfg,
        open_position,
        move_pose,
        move_pose_linear,
        set_gripper,
        close,
    ):
        plan = self._fresh_grasp6d_plan(gcfg)
        if plan is None:
            self.set_state(GraspStages.FAILED, 'no fresh 6D grasp plan')
            return False

        pregrasp = self._pose_array_item_as_stamped(plan, 0)
        approach = self._pose_array_item_as_stamped(plan, 1)
        grasp = self._pose_array_item_as_stamped(plan, 2)
        lift = self._pose_array_item_as_stamped(plan, 3)
        locked_obj = deepcopy(getattr(self, 'latest_grasp6d_plan_object', None))

        if not self._simulate_grasp6d_plan_if_required(gcfg, gripper_cfg, plan):
            return False

        retargeted = self._visual_retarget_6d_poses(
            locked_obj,
            [pregrasp, approach, grasp, lift],
            gcfg,
            'execution start',
            required=bool(gcfg.get('visual_retarget_require_start_detection', True)),
        )
        if retargeted is None:
            return False
        (pregrasp, approach, grasp, lift), locked_obj = retargeted

        self.set_state(GraspStages.PLAN_PREGRASP, 'using 6D grasp plan')
        if not self._command_gripper_position(
            set_gripper,
            open_position,
            'open gripper before 6D motion',
            self._cfg_float(gripper_cfg, 'open_wait_sec', 0.5),
        ):
            return False
        if not self._plan_and_execute_pose(GraspStages.MOVE_PREGRASP, '6D pregrasp', pregrasp, move_pose, '6D pregrasp'):
            return False

        retargeted = self._visual_retarget_6d_poses(
            locked_obj,
            [approach, grasp, lift],
            gcfg,
            'pregrasp',
            required=bool(gcfg.get('visual_retarget_require_pregrasp_detection', True)),
        )
        if retargeted is None:
            return False
        (approach, grasp, lift), locked_obj = retargeted

        if not self._plan_and_execute_pose(
            GraspStages.APPROACH_TARGET,
            'linear 6D approach',
            approach,
            move_pose_linear,
            '6D approach',
        ):
            return False

        if bool(gcfg.get('visual_retarget_after_approach', True)):
            retargeted = self._visual_retarget_6d_poses(
                locked_obj,
                [grasp, lift],
                gcfg,
                'approach',
                required=bool(gcfg.get('visual_retarget_require_approach_detection', False)),
            )
            if retargeted is None:
                return False
            (grasp, lift), locked_obj = retargeted

        if not self._plan_and_execute_pose(
            GraspStages.APPROACH_TARGET,
            'linear 6D grasp pose',
            grasp,
            move_pose_linear,
            '6D grasp pose',
        ):
            return False

        close_label = 'force-guided close' if bool(gripper_cfg.get('use_compliant_close', True)) else 'fixed gripper close'
        self.set_state(GraspStages.COMPLIANT_CLOSE, close_label)
        ok, message = self._close_gripper(gripper_cfg, set_gripper, close)
        if not ok:
            self.set_state(GraspStages.FAILED, message)
            return False

        if not self._plan_and_execute_pose(
            GraspStages.LIFT_OBJECT,
            'linear 6D lift',
            lift,
            move_pose_linear,
            '6D lift',
        ):
            return False
        self.set_state(GraspStages.SUCCESS, '6D grasp done', True)
        return True

    def _plan_and_execute_pose(self, stage, label, pose, move_pose, settle_reason):
        self.set_state(stage, 'planning ' + label)
        resp = move_pose(pose, False)
        if not resp.success:
            self.set_state(GraspStages.FAILED, '%s planning failed: %s' % (label, resp.message))
            return False
        if (
            is_position_only_fallback_message(getattr(resp, 'message', ''))
            and not self._position_only_execute_allowed()
        ):
            self.set_state(
                GraspStages.FAILED,
                position_only_rejection_message(label, getattr(resp, 'message', '')),
            )
            return False
        if (
            is_orientation_fallback_message(getattr(resp, 'message', ''))
            and not self._orientation_fallback_execute_allowed()
        ):
            self.set_state(
                GraspStages.FAILED,
                orientation_fallback_rejection_message(label, getattr(resp, 'message', '')),
            )
            return False
        self.set_state(stage, 'moving ' + label)
        resp = move_pose(pose, True)
        if not resp.success:
            self.set_state(GraspStages.FAILED, '%s failed: %s' % (label, resp.message))
            return False
        settled = self._wait_for_motion_settle(settle_reason)
        if settled is False:
            self.set_state(
                GraspStages.FAILED,
                '%s feedback did not settle; refusing to overlap the next motion' % label,
            )
            return False
        return True

    def _visual_retarget_6d_poses(self, reference_obj, poses, gcfg, stage_label, required=False):
        if not bool(gcfg.get('visual_retarget_enabled', False)):
            return list(poses), reference_obj
        if reference_obj is None or not bool(getattr(reference_obj, 'detected', False)):
            message = 'visual retarget unavailable: no object locked with the 6D plan'
            if required:
                self.set_state(GraspStages.FAILED, message)
                return None
            rospy.logwarn('%s', message)
            return list(poses), reference_obj

        live_obj = self._wait_for_stable_visual_target(reference_obj, gcfg, stage_label)
        if live_obj is None:
            message = 'target not visible/stable after %s; keeping last trusted 6D pose' % stage_label
            if required:
                self.set_state(GraspStages.FAILED, message)
                return None
            rospy.logwarn('%s', message)
            return list(poses), reference_obj

        delta = self._object_delta_xyz(reference_obj, live_obj)
        distance = math.sqrt(sum(float(value) ** 2 for value in delta))
        max_correction = max(0.0, self._cfg_float(gcfg, 'visual_retarget_max_correction_m', 0.04))
        if max_correction > 0.0 and distance > max_correction:
            self.set_state(
                GraspStages.FAILED,
                (
                    'visual/hand-eye consistency failed after %s: target shifted %.3fm > %.3fm; '
                    'check rigid camera mounting and hand-eye calibration'
                ) % (stage_label, distance, max_correction),
            )
            return None

        deadband = max(0.0, self._cfg_float(gcfg, 'visual_retarget_deadband_m', 0.002))
        shifted = [self._translate_pose(pose, delta if distance > deadband else (0.0, 0.0, 0.0)) for pose in poses]
        rospy.loginfo(
            'Grasp visual retarget after %s: delta=(%.3f, %.3f, %.3f)m norm=%.3fm orientation_preserved=1',
            stage_label,
            float(delta[0]),
            float(delta[1]),
            float(delta[2]),
            distance,
        )
        self.set_state(
            GraspStages.APPROACH_TARGET,
            'visual target locked after %s; correction %.1f mm' % (stage_label, distance * 1000.0),
        )
        return shifted, live_obj

    def _wait_for_stable_visual_target(self, reference_obj, gcfg, stage_label):
        timeout = max(0.0, self._cfg_float(gcfg, 'visual_retarget_timeout_sec', 1.2))
        sample_period = max(0.02, self._cfg_float(gcfg, 'visual_retarget_sample_sec', 0.05))
        required = max(1, int(gcfg.get('visual_retarget_required_samples', 3)))
        max_jitter = max(0.0, self._cfg_float(gcfg, 'visual_retarget_max_jitter_m', 0.012))
        raw_max_age = max(0.0, self._cfg_float(gcfg, 'visual_retarget_raw_max_age_sec', 0.30))
        min_confidence = max(
            0.0,
            min(1.0, self._cfg_float(gcfg, 'visual_retarget_min_object_confidence', 0.35)),
        )
        samples = []
        last_token = None
        start = time.monotonic()

        while self.active and time.monotonic() - start < timeout and not rospy.is_shutdown():
            obj = getattr(self, 'latest_visual_obj', None)
            obj_time = getattr(self, 'latest_visual_obj_time', None)
            if obj is None or obj_time is None:
                obj = getattr(self, 'latest_obj', None)
                obj_time = getattr(self, 'latest_obj_time', None)
            token = self._time_token(obj_time)
            if (
                obj is not None
                and bool(getattr(obj, 'detected', False))
                and float(getattr(obj, 'confidence', 1.0) or 0.0) >= min_confidence
                and token is not None
                and token != last_token
                and self._raw_detection_is_fresh(raw_max_age)
                and self._same_target_label(reference_obj, obj)
            ):
                last_token = token
                xyz = self._object_xyz(obj)
                if xyz is not None:
                    samples.append((xyz, deepcopy(obj)))
                    samples = samples[-required:]
                    if len(samples) >= required:
                        center = tuple(
                            sorted(float(item[0][axis]) for item in samples)[len(samples) // 2]
                            for axis in range(3)
                        )
                        spread = max(
                            math.sqrt(sum((float(item[0][axis]) - center[axis]) ** 2 for axis in range(3)))
                            for item in samples
                        )
                        if spread <= max_jitter:
                            result = samples[-1][1]
                            point = result.pose_base.pose.position
                            point.x, point.y, point.z = center
                            rospy.loginfo(
                                (
                                    'Grasp live target after %s: xyz=(%.3f, %.3f, %.3f) '
                                    'samples=%d spread=%.3fm confidence=%.3f'
                                ),
                                stage_label,
                                center[0],
                                center[1],
                                center[2],
                                len(samples),
                                spread,
                                float(getattr(result, 'confidence', 1.0) or 0.0),
                            )
                            return result
                        samples = samples[-max(1, required - 1):]
            rospy.sleep(sample_period)
        return None

    def _raw_detection_is_fresh(self, max_age_sec):
        if not bool(getattr(self, 'latest_raw_detection', False)):
            return False
        stamp = getattr(self, 'latest_raw_detection_time', None)
        if stamp is None:
            return False
        if max_age_sec <= 0.0:
            return True
        try:
            return (rospy.Time.now() - stamp).to_sec() <= max_age_sec
        except Exception:
            return False

    @staticmethod
    def _time_token(stamp):
        if stamp is None:
            return None
        try:
            return float(stamp.to_sec())
        except Exception:
            return id(stamp)

    @staticmethod
    def _same_target_label(first, second):
        first_label = str(getattr(first, 'label', '') or '').strip().lower()
        second_label = str(getattr(second, 'label', '') or '').strip().lower()
        return not first_label or not second_label or first_label == second_label

    @staticmethod
    def _object_xyz(obj):
        try:
            p = obj.pose_base.pose.position
            return float(p.x), float(p.y), float(p.z)
        except Exception:
            return None

    def _object_delta_xyz(self, first, second):
        a = self._object_xyz(first)
        b = self._object_xyz(second)
        if a is None or b is None:
            return 0.0, 0.0, 0.0
        return tuple(float(b[index]) - float(a[index]) for index in range(3))

    @staticmethod
    def _translate_pose(pose, delta_xyz):
        shifted = deepcopy(pose)
        shifted.pose.position.x += float(delta_xyz[0])
        shifted.pose.position.y += float(delta_xyz[1])
        shifted.pose.position.z += float(delta_xyz[2])
        return shifted

    def _fresh_grasp6d_plan(self, gcfg):
        plan = getattr(self, 'latest_grasp6d_plan', None)
        if plan is None or len(getattr(plan, 'poses', [])) < 4:
            return None
        plan_time = getattr(self, 'latest_grasp6d_plan_time', None)
        if plan_time is None:
            return None
        max_age = max(0.0, self._cfg_float(gcfg, 'grasp6d_plan_max_age_sec', 2.0))
        try:
            age = (rospy.Time.now() - plan_time).to_sec()
        except Exception:
            return None
        if age > max_age:
            rospy.logwarn('Rejected stale 6D grasp plan age %.2fs > %.2fs', age, max_age)
            return None
        max_drift = max(0.0, self._cfg_float(gcfg, 'grasp6d_plan_max_object_drift_m', 0.0))
        if max_drift > 0.0:
            locked_obj = getattr(self, 'latest_grasp6d_plan_object', None)
            latest_obj = getattr(self, 'latest_obj', None)
            if (
                locked_obj is not None
                and latest_obj is not None
                and bool(getattr(latest_obj, 'detected', False))
            ):
                drift = self._object_distance(locked_obj, latest_obj)
                if drift > max_drift:
                    rospy.logwarn(
                        'Rejected 6D grasp plan: object drift %.3fm > %.3fm since plan generation',
                        drift,
                        max_drift,
                    )
                    return None
        return plan

    def _simulate_grasp6d_plan_if_required(self, gcfg, gripper_cfg, plan):
        twin_cfg = rospy.get_param('/mujoco_digital_twin', {})
        if not bool(twin_cfg.get('enabled', False)) or not bool(twin_cfg.get('execution_gate_enabled', False)):
            return True

        self.set_state(GraspStages.PLAN_PREGRASP, 'MuJoCo digital twin checking 6D plan')
        require_object = bool(twin_cfg.get('require_object_pose', True))
        object_pose = getattr(self, 'latest_obj', None)
        if require_object and (object_pose is None or not bool(getattr(object_pose, 'detected', False))):
            self.set_state(GraspStages.FAILED, 'MuJoCo simulation blocked: no detected object pose')
            return False

        send_joint_state = bool(twin_cfg.get('send_joint_state_in_request', False))
        joint_state = getattr(self, 'latest_joint_state', None) if send_joint_state else None
        if send_joint_state and joint_state is None:
            self.set_state(GraspStages.FAILED, 'MuJoCo simulation blocked: no /joint_states for request payload')
            return False
        object_model = dict(twin_cfg.get('object_model', {}) or {})
        open_width = self._cfg_float(twin_cfg, 'open_width_m', self._cfg_float(gripper_cfg, 'open_position_m', 0.05))
        payload = build_simulation_payload(
            joint_state=joint_state,
            object_pose=object_pose if object_pose is not None else None,
            grasp_plan=plan,
            gripper_width_m=open_width,
            object_model=object_model,
        )
        payload['close_width_m'] = self._cfg_float(
            twin_cfg,
            'close_width_m',
            self._cfg_float(gripper_cfg, 'simple_close_position_m', self._cfg_float(gripper_cfg, 'close_limit_m', 0.0)),
        )
        payload['min_lift_success_m'] = self._cfg_float(twin_cfg, 'min_lift_success_m', 0.015)

        try:
            client = MujocoDigitalTwinClient(
                twin_cfg.get('server_url', 'http://172.23.132.97:8000'),
                timeout_sec=self._cfg_float(twin_cfg, 'timeout_sec', 20.0),
            )
            response = client.simulate_grasp(payload)
        except Exception as exc:
            message = 'MuJoCo simulation request failed: %s' % exc
            if bool(twin_cfg.get('allow_execution_on_error', False)):
                rospy.logwarn('%s; execution allowed by config', message)
                return True
            self.set_state(GraspStages.FAILED, message)
            return False

        score = int(response.get('score', 0) or 0)
        min_score = int(twin_cfg.get('min_score', 80) or 80)
        simulation_ok = bool(response.get('simulation_ok', False)) and score >= min_score
        if not simulation_ok:
            reason = str(response.get('failure_reason') or 'score %d < %d' % (score, min_score))
            self.set_state(GraspStages.FAILED, 'MuJoCo simulation blocked execution: %s' % reason)
            return False
        self.set_state(GraspStages.PLAN_PREGRASP, 'MuJoCo simulation passed score=%d' % score)
        return True

    @staticmethod
    def _pose_array_item_as_stamped(plan, index):
        pose = PoseStamped()
        pose.header = plan.header
        pose.pose = deepcopy(plan.poses[index])
        return pose

    def _current_tool_pose_base(self):
        tf_buffer = getattr(self, 'tf_buffer', None)
        if tf_buffer is None:
            return None
        base_frame = str(rospy.get_param('/handeye/base_frame', 'base_link'))
        tool_frame = str(rospy.get_param('/handeye/parent_frame', 'tool0'))
        timeout = float(rospy.get_param('/handeye/tf_timeout_sec', 0.2))
        try:
            transform = tf_buffer.lookup_transform(
                base_frame,
                tool_frame,
                rospy.Time(0),
                rospy.Duration(timeout),
            )
        except Exception as exc:
            rospy.logwarn_throttle(2.0, 'Grasp tool TF lookup failed: %s', exc)
            return None
        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    @staticmethod
    def _pose_distance(first, second):
        if first is None or second is None:
            return float('inf')
        try:
            a = first.pose.position
            b = second.pose.position
            return math.sqrt(
                (float(a.x) - float(b.x)) ** 2
                + (float(a.y) - float(b.y)) ** 2
                + (float(a.z) - float(b.z)) ** 2
            )
        except Exception:
            return float('inf')

    def _pose_close_enough(self, first, second, tolerance_m):
        return self._pose_distance(first, second) <= max(0.0, float(tolerance_m))

    @staticmethod
    def _cfg_float(cfg, key, default):
        try:
            if isinstance(cfg, dict) and key in cfg:
                return float(cfg.get(key))
        except Exception:
            pass
        return float(default)

    def _object_distance(self, first, second):
        try:
            return self._pose_distance(first.pose_base, second.pose_base)
        except Exception:
            return float('inf')

    def _target_for_approach(self, locked_obj, gcfg):
        latest = getattr(self, 'latest_obj', None)
        if latest is None:
            return deepcopy(locked_obj)
        max_refine = self._cfg_float(gcfg, 'max_locked_target_refine_m', 0.06)
        distance = self._object_distance(locked_obj, latest)
        if distance <= max(0.0, max_refine):
            if distance > 1e-6:
                rospy.loginfo('Grasp refined locked target by %.3f m before approach', distance)
            return deepcopy(latest)
        rospy.logwarn(
            'Grasp kept locked target; latest detection jumped %.3f m > %.3f m',
            distance,
            max_refine,
        )
        return deepcopy(locked_obj)

    def _log_object_pose(self, label, obj):
        try:
            p = obj.pose_base.pose.position
            rospy.loginfo(
                'Grasp %s: xyz=(%.3f, %.3f, %.3f) confidence=%.3f',
                label,
                float(p.x),
                float(p.y),
                float(p.z),
                float(getattr(obj, 'confidence', 1.0) or 0.0),
            )
        except Exception:
            rospy.loginfo('Grasp %s: pose unavailable', label)

    def _wait_for_motion_settle(self, reason='motion'):
        timeout = max(0.0, float(rospy.get_param('/grasp/motion_settle_timeout_sec', 2.0)))
        min_sec = max(0.0, float(rospy.get_param('/grasp/motion_settle_min_sec', 0.8)))
        sample_period = max(0.02, float(rospy.get_param('/grasp/motion_settle_sample_sec', 0.05)))
        epsilon = max(0.0, float(rospy.get_param('/grasp/motion_settle_position_epsilon_rad', 0.0025)))
        required = max(1, int(rospy.get_param('/grasp/motion_settle_required_samples', 3)))
        if timeout <= 0.0:
            return True

        start = time.monotonic()
        previous = self._joint_positions_tuple()
        stable_count = 0
        if previous is None:
            rospy.logwarn_throttle(2.0, 'Grasp settle fallback sleep: no joint state for %s', reason)
            rospy.sleep(min_sec if min_sec > 0.0 else min(timeout, 0.2))
            return False

        while self.active and time.monotonic() - start < timeout and not rospy.is_shutdown():
            rospy.sleep(sample_period)
            current = self._joint_positions_tuple()
            elapsed = time.monotonic() - start
            if current is None:
                stable_count = 0
                continue
            max_delta = max(abs(float(a) - float(b)) for a, b in zip(previous, current))
            if max_delta <= epsilon and elapsed >= min_sec:
                stable_count += 1
                if stable_count >= required:
                    rospy.loginfo('Grasp motion settled after %.2fs for %s', elapsed, reason)
                    return True
            else:
                stable_count = 0
            previous = current

        rospy.logwarn('Grasp motion settle timeout after %.2fs for %s', time.monotonic() - start, reason)
        return False

    @staticmethod
    def _position_only_execute_allowed():
        return bool(rospy.get_param('/robot/position_only_execute_enabled', False))

    @staticmethod
    def _orientation_fallback_execute_allowed():
        return bool(rospy.get_param('/grasp/accept_orientation_fallback', False))

    def _command_gripper_position(self, set_gripper, position, label, wait_sec):
        try:
            resp = set_gripper(float(position))
        except Exception as exc:
            self.set_state(GraspStages.FAILED, '%s failed: %s' % (label, exc))
            return False
        if not bool(getattr(resp, 'success', True)):
            self.set_state(GraspStages.FAILED, '%s failed: %s' % (label, getattr(resp, 'message', '')))
            return False
        delay = max(0.0, float(wait_sec))
        if delay > 0.0:
            rospy.sleep(delay)
        return True

    def _close_gripper(self, gripper_cfg, set_gripper, close):
        if bool(gripper_cfg.get('use_compliant_close', True)):
            if close is None:
                return False, 'compliant close service is unavailable'
            resp = close(True)
            return bool(resp.success), getattr(resp, 'message', '')

        close_position = self._cfg_float(
            gripper_cfg,
            'simple_close_position_m',
            self._cfg_float(gripper_cfg, 'close_limit_m', 0.05),
        )
        wait_sec = self._cfg_float(gripper_cfg, 'simple_close_wait_sec', 0.8)
        ok = self._command_gripper_position(set_gripper, close_position, 'fixed gripper close', wait_sec)
        return ok, 'fixed gripper close command published' if ok else 'fixed gripper close failed'

    def _joint_positions_tuple(self):
        msg = getattr(self, 'latest_joint_state', None)
        positions = getattr(msg, 'position', None)
        if not positions:
            return None
        try:
            return tuple(float(v) for v in positions)
        except Exception:
            return None

    def _lookup_camera_pose_base(self):
        tf_buffer = getattr(self, 'tf_buffer', None)
        if tf_buffer is None:
            return None
        base_frame = str(rospy.get_param('/handeye/base_frame', 'base_link'))
        camera_frame = str(rospy.get_param('/handeye/camera_frame', rospy.get_param('/camera/frame_id', 'camera_link')))
        timeout = float(rospy.get_param('/handeye/tf_timeout_sec', 0.2))
        try:
            transform = tf_buffer.lookup_transform(
                base_frame,
                camera_frame,
                rospy.Time(0),
                rospy.Duration(timeout),
            )
        except Exception as exc:
            rospy.logwarn_throttle(2.0, 'Grasp camera TF lookup failed: %s', exc)
            return None
        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

if __name__ == '__main__':
    rospy.init_node('grasp_task_node')
    GraspTaskNode()
    rospy.spin()
