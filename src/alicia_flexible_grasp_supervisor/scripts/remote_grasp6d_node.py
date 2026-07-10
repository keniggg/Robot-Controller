#!/usr/bin/env python3
import math
import threading

import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf.transformations import quaternion_from_matrix, quaternion_matrix

try:
    import tf2_ros
except Exception:
    tf2_ros = None

from alicia_flexible_grasp.grasp.grasp6d_sequence import make_grasp_sequence_from_grasp_pose
from alicia_flexible_grasp.robot.planning_feedback import (
    is_orientation_fallback_message,
    is_position_only_fallback_message,
)
from alicia_flexible_grasp.vision.grasp6d_adapter import CameraIntrinsics
from alicia_flexible_grasp.vision.pose_estimator import PoseEstimator
from alicia_flexible_grasp.vision.remote_grasp6d_client import RemoteGrasp6DClient, RemoteGraspCandidate
from alicia_flexible_grasp_supervisor.msg import ObjectPose
from alicia_flexible_grasp_supervisor.srv import SetTargetPose, TriggerZero, TriggerZeroResponse


OPTICAL_TO_ROS_CAMERA = np.asarray(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ],
    dtype=float,
)


class LatestRgbdBuffer:
    def __init__(self):
        self._lock = threading.Lock()
        self.color = None
        self.depth = None
        self.stamp = None
        self.frame_id = ''
        self.color_seq = 0
        self.consumed_seq = 0

    def update_color(self, color, stamp, frame_id):
        with self._lock:
            self.color = color
            self.stamp = stamp
            self.frame_id = frame_id
            self.color_seq += 1

    def update_depth(self, depth):
        with self._lock:
            self.depth = depth

    def take_latest(self):
        with self._lock:
            if self.color_seq == self.consumed_seq:
                return None
            self.consumed_seq = self.color_seq
            return self._snapshot_locked()

    def snapshot_latest(self):
        with self._lock:
            return self._snapshot_locked()

    def _snapshot_locked(self):
        if self.color is None or self.depth is None:
            return None
        return self.color.copy(), self.depth.copy(), self.stamp, self.frame_id


def normalize_candidate_frame_convention(convention):
    value = str(convention or 'opencv_optical').strip().lower()
    aliases = {
        'ros': 'ros_camera_link',
        'camera_link': 'ros_camera_link',
        'ros_link': 'ros_camera_link',
        'ros_camera': 'ros_camera_link',
        'opencv': 'opencv_optical',
        'optical': 'opencv_optical',
        'camera_optical': 'opencv_optical',
        'color_optical': 'opencv_optical',
    }
    return aliases.get(value, value)


def convert_candidate_to_camera_link(candidate, convention='opencv_optical'):
    convention = normalize_candidate_frame_convention(convention)
    if convention == 'ros_camera_link':
        return candidate
    if convention != 'opencv_optical':
        raise ValueError('unknown remote grasp candidate frame convention: %s' % convention)

    translation = OPTICAL_TO_ROS_CAMERA.dot(np.asarray(candidate.translation_m, dtype=float))
    optical_from_grasp = quaternion_matrix(np.asarray(candidate.quaternion_xyzw, dtype=float))
    camera_from_grasp = np.eye(4, dtype=float)
    camera_from_grasp[:3, :3] = OPTICAL_TO_ROS_CAMERA.dot(optical_from_grasp[:3, :3])
    quaternion = _normalize_quaternion(np.asarray(quaternion_from_matrix(camera_from_grasp), dtype=float))
    return RemoteGraspCandidate(
        score=float(candidate.score),
        translation_m=translation,
        quaternion_xyzw=quaternion,
        width_m=float(candidate.width_m),
    )


def _normalize_quaternion(quaternion):
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-12:
        raise ValueError('remote grasp candidate quaternion has zero norm')
    quaternion = quaternion / norm
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    return quaternion


def expanded_bbox_roi(image_shape, bbox_x, bbox_y, bbox_width, bbox_height, margin_px=0):
    height, width = image_shape[:2]
    margin = max(0, int(margin_px))
    x0 = max(0, int(bbox_x) - margin)
    y0 = max(0, int(bbox_y) - margin)
    x1 = min(int(width), int(bbox_x) + max(0, int(bbox_width)) + margin)
    y1 = min(int(height), int(bbox_y) + max(0, int(bbox_height)) + margin)
    if x1 <= x0 or y1 <= y0:
        raise ValueError('invalid target ROI bbox')
    return x0, y0, x1, y1


def mask_depth_to_roi(depth, roi):
    x0, y0, x1, y1 = roi
    masked = np.zeros_like(depth)
    masked[y0:y1, x0:x1] = depth[y0:y1, x0:x1]
    return masked


def valid_depth_count(depth):
    values = np.asarray(depth)
    if np.issubdtype(values.dtype, np.floating):
        return int(np.count_nonzero(np.isfinite(values) & (values > 0.0)))
    return int(np.count_nonzero(values > 0))


def make_remote_pose_estimator(cam_cfg, hcfg, gcfg, tf2_module=None):
    tf_module = tf2_module if tf2_module is not None else tf2_ros
    tf_buffer = None
    tf_listener = None
    if bool(hcfg.get('use_tf', True)) and tf_module is not None:
        tf_buffer = tf_module.Buffer()
        tf_listener = tf_module.TransformListener(tf_buffer)
    elif bool(hcfg.get('use_tf', True)):
        rospy.logwarn('tf2_ros is unavailable; remote 6D grasp will use configured static handeye transform')

    pose_estimator = PoseEstimator(
        hcfg.get('camera_frame', cam_cfg.get('frame_id', 'camera_link')),
        hcfg.get('base_frame', 'base_link'),
        hcfg.get('translation_xyz', [0.0, 0.0, 0.0]),
        hcfg.get('rotation_xyzw', [0.0, 0.0, 0.0, 1.0]),
        gcfg.get('default_orientation_xyzw', [0.0, 0.7071, 0.0, 0.7071]),
        tf_buffer=tf_buffer,
        tf_timeout_sec=hcfg.get('tf_timeout_sec', 0.2),
        tf_lookup_latest=hcfg.get('tf_lookup_latest', True),
        allow_static_fallback=hcfg.get('allow_static_fallback', True),
    )
    return pose_estimator, tf_buffer, tf_listener


def select_first_reachable_candidate(
    candidates,
    pose_estimator,
    reachability_fn,
    stamp,
    camera_frame,
    candidate_frame_convention='opencv_optical',
    candidate_filter_fn=None,
    candidate_rank_fn=None,
):
    ranked = []
    for candidate in candidates:
        camera_candidate = convert_candidate_to_camera_link(candidate, candidate_frame_convention)
        pose = pose_estimator.make_base_pose_from_camera_pose(
            camera_candidate.translation_m,
            camera_candidate.quaternion_xyzw,
            stamp=stamp,
            camera_frame=camera_frame,
        )
        if candidate_filter_fn is not None and not bool(candidate_filter_fn(candidate, camera_candidate, pose)):
            continue
        if bool(reachability_fn(pose)):
            if candidate_rank_fn is None:
                return camera_candidate, pose
            try:
                rank = float(candidate_rank_fn(candidate, camera_candidate, pose))
            except Exception:
                rank = float('inf')
            ranked.append((rank, -float(getattr(camera_candidate, 'score', 0.0)), camera_candidate, pose))
    if ranked:
        ranked.sort(key=lambda item: (item[0], item[1]))
        return ranked[0][2], ranked[0][3]
    return None, None


def make_grasp_plan_pose_array(grasp_pose, stamp, grasp_config):
    plan = make_grasp_sequence_from_grasp_pose(
        grasp_pose,
        pregrasp_distance_m=float(grasp_config.get('pregrasp_distance_m', 0.08)),
        approach_offset_m=float(grasp_config.get('final_approach_offset_m', 0.015)),
        lift_height_m=float(grasp_config.get('lift_height_m', 0.05)),
    )
    msg = PoseArray()
    msg.header.frame_id = grasp_pose.header.frame_id
    msg.header.stamp = stamp if stamp is not None else rospy.Time(0)
    msg.poses = [plan.pregrasp.pose, plan.approach.pose, plan.grasp.pose, plan.lift.pose]
    return msg


class RemoteGrasp6DNode:
    def __init__(self):
        self.enabled = bool(rospy.get_param('/grasp_6d/enabled', True))
        self.bridge = CvBridge()
        self.frames = LatestRgbdBuffer()
        self.latest_object = None
        self.latest_object_time = None
        self._object_lock = threading.Lock()
        self.last_error = ''
        self._request_lock = threading.Lock()
        self._backoff_until = rospy.Time(0)
        self.plan_pub = rospy.Publisher(rospy.get_param('/grasp/grasp6d_plan_topic', '/grasp_6d/plan'), PoseArray, queue_size=1)
        self.status_pub = rospy.Publisher('/grasp_6d/status', String, queue_size=1, latch=True)

        cam_cfg = rospy.get_param('/camera', {})
        hcfg = rospy.get_param('/handeye', {})
        gcfg = rospy.get_param('/grasp', {})
        remote_cfg = rospy.get_param('/grasp_6d/remote', {})
        server_url = rospy.get_param('/grasp_6d/remote/server_url', remote_cfg.get('server_url', 'http://172.23.132.97:8000'))
        timeout_sec = float(rospy.get_param('/grasp_6d/remote/timeout_sec', remote_cfg.get('timeout_sec', 3.0)))
        self.max_candidates = int(rospy.get_param('/grasp_6d/remote/max_candidates', remote_cfg.get('max_candidates', 20)))
        self.auto_request = bool(rospy.get_param('/grasp_6d/remote/auto_request', remote_cfg.get('auto_request', False)))
        self.failure_backoff_sec = max(0.0, float(rospy.get_param('/grasp_6d/remote/failure_backoff_sec', remote_cfg.get('failure_backoff_sec', 8.0))))
        self.candidate_frame_convention = normalize_candidate_frame_convention(
            rospy.get_param('/grasp_6d/remote/candidate_frame_convention', remote_cfg.get('candidate_frame_convention', 'opencv_optical'))
        )
        self.allow_position_only_fallback = bool(
            rospy.get_param(
                '/grasp_6d/remote/accept_position_only_fallback',
                remote_cfg.get(
                    'accept_position_only_fallback',
                    rospy.get_param('/robot/position_only_execute_enabled', False),
                ),
            )
        )
        self.allow_orientation_fallback = bool(
            rospy.get_param(
                '/grasp_6d/remote/accept_orientation_fallback',
                remote_cfg.get('accept_orientation_fallback', False),
            )
        )
        self._position_only_rejected_count = 0
        self._orientation_fallback_rejected_count = 0
        self.use_perception_roi = bool(
            rospy.get_param('/grasp_6d/remote/use_perception_roi', remote_cfg.get('use_perception_roi', True))
        )
        self.require_perception_roi = bool(
            rospy.get_param('/grasp_6d/remote/require_perception_roi', remote_cfg.get('require_perception_roi', True))
        )
        self.roi_margin_px = int(
            rospy.get_param('/grasp_6d/remote/perception_roi_margin_px', remote_cfg.get('perception_roi_margin_px', 20))
        )
        self.roi_max_age_sec = max(
            0.0,
            float(rospy.get_param('/grasp_6d/remote/perception_roi_max_age_sec', remote_cfg.get('perception_roi_max_age_sec', 1.0))),
        )
        self.roi_min_valid_depth_px = max(
            1,
            int(rospy.get_param('/grasp_6d/remote/perception_roi_min_valid_depth_px', remote_cfg.get('perception_roi_min_valid_depth_px', 250))),
        )
        self.candidate_target_gate_enabled = bool(
            rospy.get_param(
                '/grasp_6d/remote/candidate_target_gate_enabled',
                remote_cfg.get('candidate_target_gate_enabled', True),
            )
        )
        self.candidate_max_target_distance_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/candidate_max_target_distance_m',
                    remote_cfg.get('candidate_max_target_distance_m', 0.04),
                )
            ),
        )
        self.candidate_min_relative_z_m = float(
            rospy.get_param(
                '/grasp_6d/remote/candidate_min_relative_z_m',
                remote_cfg.get('candidate_min_relative_z_m', -0.015),
            )
        )
        self.candidate_max_relative_z_m = float(
            rospy.get_param(
                '/grasp_6d/remote/candidate_max_relative_z_m',
                remote_cfg.get('candidate_max_relative_z_m', 0.08),
            )
        )
        self._target_gate_rejected_count = 0
        try:
            self.client = RemoteGrasp6DClient(server_url, timeout_sec=timeout_sec)
        except ValueError as exc:
            rospy.logfatal('invalid remote 6D grasp server URL: %s', exc)
            raise
        self.pose_estimator, self.tf_buffer, self.tf_listener = make_remote_pose_estimator(cam_cfg, hcfg, gcfg)

        rospy.Subscriber(cam_cfg.get('color_topic', '/supervisor/camera/color/image_raw'), Image, self.color_cb, queue_size=1, buff_size=2**24, tcp_nodelay=True)
        rospy.Subscriber(cam_cfg.get('depth_topic', '/supervisor/camera/depth/image_raw'), Image, self.depth_cb, queue_size=1, buff_size=2**24, tcp_nodelay=True)
        rospy.Subscriber(
            rospy.get_param('/perception/output_object_topic', '/perception/object'),
            ObjectPose,
            self.object_cb,
            queue_size=1,
        )
        self.rate_hz = max(0.1, float(rospy.get_param('/grasp_6d/remote/request_hz', remote_cfg.get('request_hz', rospy.get_param('/grasp_6d/plan_hz', 1.0)))))
        rospy.Service('/grasp_6d/request_plan', TriggerZero, self.request_plan_cb)
        self._check_remote_health()
        mode = 'auto %.2f Hz' % self.rate_hz if self.auto_request else 'manual trigger'
        self.status_pub.publish(String('remote 6D grasp waiting for RGB-D: %s (%s)' % (server_url, mode)))

    def color_cb(self, msg):
        self.frames.update_color(self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'), msg.header.stamp, msg.header.frame_id)

    def depth_cb(self, msg):
        self.frames.update_depth(self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'))

    def object_cb(self, msg):
        now = rospy.Time.now()
        if not bool(getattr(msg, 'detected', False)):
            with self._object_lock:
                self.latest_object = msg
                self.latest_object_time = now
            return

        gcfg = rospy.get_param('/grasp', {})
        confidence = float(getattr(msg, 'confidence', 1.0) or 0.0)
        min_confidence = self._cfg_float(gcfg, 'min_object_confidence', 0.50)
        if confidence < min_confidence:
            rospy.logwarn_throttle(
                1.0,
                'remote 6D ignored low-confidence object %.3f < %.3f',
                confidence,
                min_confidence,
            )
            return

        with self._object_lock:
            previous = self.latest_object
            previous_time = self.latest_object_time
        max_jump = self._cfg_float(gcfg, 'max_object_jump_m', 0.12)
        jump_window = self._cfg_float(gcfg, 'object_jump_filter_window_sec', 4.0)
        if (
            previous is not None
            and bool(getattr(previous, 'detected', False))
            and previous_time is not None
            and max_jump > 0.0
        ):
            try:
                age = (now - previous_time).to_sec()
            except Exception:
                age = float('inf')
            jump = self._object_pose_distance(previous, msg)
            if age <= jump_window and jump > max_jump:
                rospy.logwarn_throttle(
                    1.0,
                    'remote 6D ignored object jump %.3f m > %.3f m within %.1fs',
                    jump,
                    max_jump,
                    jump_window,
                )
                return

        with self._object_lock:
            self.latest_object = msg
            self.latest_object_time = now

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.enabled and self.auto_request:
                self._process_latest_frame()
            rate.sleep()

    def request_plan_cb(self, req):
        if not bool(getattr(req, 'trigger', False)):
            return TriggerZeroResponse(False, 'trigger=false')
        if not self.enabled:
            return TriggerZeroResponse(False, 'remote 6D disabled')
        frame = self.frames.snapshot_latest()
        if frame is None:
            message = 'waiting for synchronized RGB-D frame'
            self.status_pub.publish(String('remote 6D request ignored: ' + message))
            return TriggerZeroResponse(False, message)
        ok, message = self._process_frame(frame, manual=True)
        return TriggerZeroResponse(bool(ok), str(message))

    def _process_latest_frame(self):
        if rospy.Time.now() < self._backoff_until:
            return
        frame = self.frames.take_latest()
        if frame is None:
            return
        self._process_frame(frame, manual=False)

    def _process_frame(self, frame, manual=False):
        if not self._request_lock.acquire(False):
            message = 'remote 6D request already running'
            if manual:
                self.status_pub.publish(String(message))
            return False, message
        color, depth, stamp, frame_id = frame
        try:
            depth_for_remote, roi_message = self._depth_for_remote(depth)
            status = 'remote 6D requesting candidates...'
            if roi_message:
                status += ' ' + roi_message
            self.status_pub.publish(String(status))
            candidates = self.client.predict(
                color,
                depth_for_remote,
                self._camera_intrinsics(),
                frame_id=frame_id or self.pose_estimator.camera_frame,
                stamp_sec=stamp.to_sec() if stamp is not None else 0.0,
                max_candidates=self.max_candidates,
            )
            self._position_only_rejected_count = 0
            self._orientation_fallback_rejected_count = 0
            self._target_gate_rejected_count = 0
            selected, grasp_pose = select_first_reachable_candidate(
                candidates,
                self.pose_estimator,
                self._plan_reachable,
                stamp=stamp,
                camera_frame=frame_id or self.pose_estimator.camera_frame,
                candidate_frame_convention=self.candidate_frame_convention,
                candidate_filter_fn=self._candidate_matches_target,
                candidate_rank_fn=self._candidate_target_distance,
            )
            if selected is None:
                if self._target_gate_rejected_count > 0:
                    message = (
                        'remote 6D returned %d candidates; %d rejected because they were not close '
                        'to the locked detection target'
                        % (len(candidates), self._target_gate_rejected_count)
                    )
                elif self._position_only_rejected_count > 0:
                    message = (
                        'remote 6D returned %d candidates; %d only reached by position-only fallback '
                        'and were rejected because they are not executable'
                        % (len(candidates), self._position_only_rejected_count)
                    )
                elif self._orientation_fallback_rejected_count > 0:
                    message = (
                        'remote 6D returned %d candidates; %d only reached by candidate orientation '
                        'fallback and were rejected because the 6D grasp orientation is not executable'
                        % (len(candidates), self._orientation_fallback_rejected_count)
                    )
                else:
                    message = 'remote 6D returned %d candidates, none reachable' % len(candidates)
                self._publish_error(message)
                return False, message
            plan_msg = make_grasp_plan_pose_array(grasp_pose, stamp, rospy.get_param('/grasp', {}))
            self.plan_pub.publish(plan_msg)
            message = 'remote 6D plan ready score=%.3f width=%.3f target_delta=%.3fm' % (
                selected.score,
                selected.width_m,
                self._candidate_target_distance(None, None, grasp_pose),
            )
            self.status_pub.publish(String(message))
            self.last_error = ''
            self._backoff_until = rospy.Time(0)
            return True, message
        except Exception as exc:
            message = 'remote 6D planning failed: %s' % exc
            self._publish_error(message)
            if self.failure_backoff_sec > 0.0:
                self._backoff_until = rospy.Time.now() + rospy.Duration(self.failure_backoff_sec)
            return False, message
        finally:
            self._request_lock.release()

    def _depth_for_remote(self, depth):
        if not self.use_perception_roi:
            return depth, ''
        try:
            obj, obj_time = self._latest_object_snapshot()
            masked, roi, valid = self._masked_depth_for_object(depth, obj, obj_time)
            return masked, 'target ROI x=%d y=%d w=%d h=%d valid_depth=%d' % (
                roi[0],
                roi[1],
                roi[2] - roi[0],
                roi[3] - roi[1],
                valid,
            )
        except Exception as exc:
            message = 'remote 6D waiting for target ROI: %s' % exc
            if self.require_perception_roi:
                raise RuntimeError(message)
            rospy.logwarn_throttle(2.0, '%s; falling back to full RGB-D frame', message)
            return depth, ''

    def _latest_object_snapshot(self):
        with self._object_lock:
            return self.latest_object, self.latest_object_time

    @staticmethod
    def _cfg_float(cfg, key, default):
        try:
            if isinstance(cfg, dict) and key in cfg:
                return float(cfg.get(key))
        except Exception:
            pass
        return float(default)

    @staticmethod
    def _object_pose_distance(first, second):
        try:
            a = first.pose_base.pose.position
            b = second.pose_base.pose.position
            return math.sqrt(
                (float(a.x) - float(b.x)) ** 2
                + (float(a.y) - float(b.y)) ** 2
                + (float(a.z) - float(b.z)) ** 2
            )
        except Exception:
            return float('inf')

    def _masked_depth_for_object(self, depth, obj, obj_time):
        if obj is None or not bool(getattr(obj, 'detected', False)):
            raise RuntimeError('no detected target object')
        if obj_time is None:
            raise RuntimeError('target object timestamp unavailable')
        age = (rospy.Time.now() - obj_time).to_sec()
        if age > self.roi_max_age_sec:
            raise RuntimeError('target ROI is stale %.2fs > %.2fs' % (age, self.roi_max_age_sec))
        roi = expanded_bbox_roi(
            np.asarray(depth).shape,
            int(getattr(obj, 'bbox_x', 0)),
            int(getattr(obj, 'bbox_y', 0)),
            int(getattr(obj, 'bbox_width', 0)),
            int(getattr(obj, 'bbox_height', 0)),
            self.roi_margin_px,
        )
        masked = mask_depth_to_roi(depth, roi)
        valid = valid_depth_count(masked)
        if valid < self.roi_min_valid_depth_px:
            raise RuntimeError('target ROI has too few valid depth pixels %d < %d' % (valid, self.roi_min_valid_depth_px))
        return masked, roi, valid

    def _candidate_matches_target(self, _candidate, _camera_candidate, grasp_pose):
        if not getattr(self, 'candidate_target_gate_enabled', True):
            return True
        obj, _obj_time = self._latest_object_snapshot()
        if obj is None or not bool(getattr(obj, 'detected', False)):
            self._target_gate_rejected_count += 1
            rospy.logwarn_throttle(1.0, 'remote 6D candidate rejected: no locked detection target')
            return False
        try:
            target = obj.pose_base.pose.position
            grasp = grasp_pose.pose.position
            dx = float(grasp.x) - float(target.x)
            dy = float(grasp.y) - float(target.y)
            dz = float(grasp.z) - float(target.z)
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        except Exception as exc:
            self._target_gate_rejected_count += 1
            rospy.logwarn_throttle(1.0, 'remote 6D candidate rejected: cannot compare target pose: %s', exc)
            return False
        max_distance = max(0.0, float(getattr(self, 'candidate_max_target_distance_m', 0.04)))
        min_z = float(getattr(self, 'candidate_min_relative_z_m', -0.015))
        max_z = float(getattr(self, 'candidate_max_relative_z_m', 0.08))
        if distance > max_distance or dz < min_z or dz > max_z:
            self._target_gate_rejected_count += 1
            rospy.logwarn_throttle(
                1.0,
                (
                    'remote 6D candidate rejected by target gate: '
                    'candidate=(%.3f, %.3f, %.3f) target=(%.3f, %.3f, %.3f) '
                    'delta=(%.3f, %.3f, %.3f) dist=%.3f limits dist<=%.3f z=[%.3f, %.3f]'
                ),
                float(grasp.x),
                float(grasp.y),
                float(grasp.z),
                float(target.x),
                float(target.y),
                float(target.z),
                dx,
                dy,
                dz,
                distance,
                max_distance,
                min_z,
                max_z,
            )
            return False
        return True

    def _candidate_target_distance(self, _candidate, _camera_candidate, grasp_pose):
        obj, _obj_time = self._latest_object_snapshot()
        if obj is None or not bool(getattr(obj, 'detected', False)):
            return float('inf')
        try:
            target = obj.pose_base.pose.position
            grasp = grasp_pose.pose.position
            dx = float(grasp.x) - float(target.x)
            dy = float(grasp.y) - float(target.y)
            dz = float(grasp.z) - float(target.z)
            return math.sqrt(dx * dx + dy * dy + dz * dz)
        except Exception:
            return float('inf')

    def _camera_intrinsics(self):
        cam_cfg = rospy.get_param('/camera', {})
        return CameraIntrinsics(
            width=int(cam_cfg.get('width', 640)),
            height=int(cam_cfg.get('height', 480)),
            fx=float(cam_cfg.get('fx', 615.0)),
            fy=float(cam_cfg.get('fy', 615.0)),
            cx=float(cam_cfg.get('cx', cam_cfg.get('width', 640) / 2.0)),
            cy=float(cam_cfg.get('cy', cam_cfg.get('height', 480) / 2.0)),
            depth_scale=float(cam_cfg.get('depth_scale', 0.001)),
        )

    def _plan_reachable(self, grasp_pose):
        try:
            rospy.wait_for_service('/supervisor/move_to_pose', timeout=0.25)
            move_pose = rospy.ServiceProxy('/supervisor/move_to_pose', SetTargetPose)
            response = move_pose(grasp_pose, False)
            if not bool(response.success):
                return False
            if is_position_only_fallback_message(getattr(response, 'message', '')) and not self.allow_position_only_fallback:
                self._position_only_rejected_count += 1
                rospy.logwarn_throttle(
                    2.0,
                    'remote 6D candidate rejected: position-only fallback is not executable: %s',
                    getattr(response, 'message', ''),
                )
                return False
            if is_orientation_fallback_message(getattr(response, 'message', '')) and not self.allow_orientation_fallback:
                self._orientation_fallback_rejected_count += 1
                rospy.logwarn_throttle(
                    2.0,
                    'remote 6D candidate rejected: candidate orientation fallback is not executable: %s',
                    getattr(response, 'message', ''),
                )
                return False
            return True
        except Exception:
            return False

    def _check_remote_health(self):
        try:
            health = self.client.health()
        except Exception as exc:
            rospy.logwarn('remote 6D health check failed: %s', exc)
            self.status_pub.publish(String('remote 6D health check failed: %s' % exc))
            return
        if bool(health.get('ok', False)):
            rospy.loginfo(
                'remote 6D server online: backend=%s loaded=%s url=%s',
                health.get('backend', 'unknown'),
                health.get('loaded', 'unknown'),
                self.client.server_url,
            )
        else:
            rospy.logwarn('remote 6D server unhealthy: %s', health)
            self.status_pub.publish(String('remote 6D server unhealthy: %s' % health))

    def _publish_error(self, message):
        if message != self.last_error:
            self.last_error = message
            rospy.logwarn('%s', message)
            self.status_pub.publish(String(message))
        else:
            rospy.logwarn_throttle(5.0, '%s', message)


if __name__ == '__main__':
    rospy.init_node('remote_grasp6d_node')
    RemoteGrasp6DNode().spin()
