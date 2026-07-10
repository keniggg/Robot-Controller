#!/usr/bin/env python3
import math
import threading
from copy import deepcopy

import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler, quaternion_from_matrix, quaternion_matrix, quaternion_multiply

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
        depth_m=getattr(candidate, 'depth_m', None),
    )


def align_candidate_to_tool_frame(candidate, model_grasp_to_tool_quaternion=None):
    """Convert GraspNet's grasp-frame orientation into the MoveIt tool frame."""
    correction = np.asarray(
        model_grasp_to_tool_quaternion
        if model_grasp_to_tool_quaternion is not None
        else [0.0, 0.0, 0.0, 1.0],
        dtype=float,
    )
    correction = _normalize_quaternion(correction)
    if np.allclose(correction, np.asarray([0.0, 0.0, 0.0, 1.0], dtype=float)):
        return candidate
    quaternion = _normalize_quaternion(
        np.asarray(quaternion_multiply(candidate.quaternion_xyzw, correction), dtype=float)
    )
    return RemoteGraspCandidate(
        score=float(candidate.score),
        translation_m=np.asarray(candidate.translation_m, dtype=float),
        quaternion_xyzw=quaternion,
        width_m=float(candidate.width_m),
        depth_m=getattr(candidate, 'depth_m', None),
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


def transform_matrix(translation_xyz, quaternion_xyzw):
    matrix = quaternion_matrix([float(value) for value in quaternion_xyzw])
    matrix[:3, 3] = np.asarray(translation_xyz, dtype=float).reshape(3)
    return matrix


def pose_matrix(pose_stamped):
    pose = pose_stamped.pose
    return transform_matrix(
        [pose.position.x, pose.position.y, pose.position.z],
        [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
    )


def project_base_target_at_tool_pose(tool_pose, target_base_xyz, tool_from_camera, intrinsics):
    base_from_camera = pose_matrix(tool_pose).dot(np.asarray(tool_from_camera, dtype=float).reshape(4, 4))
    camera_from_base = np.linalg.inv(base_from_camera)
    target_base = np.ones(4, dtype=float)
    target_base[:3] = np.asarray(target_base_xyz, dtype=float).reshape(3)
    target_camera = camera_from_base.dot(target_base)[:3]

    # camera_link follows ROS convention: x forward, y left, z up.
    optical_z = float(target_camera[0])
    if optical_z <= 1e-9:
        return float('nan'), float('nan'), optical_z
    optical_x = -float(target_camera[1])
    optical_y = -float(target_camera[2])
    u = float(intrinsics.cx) + float(intrinsics.fx) * optical_x / optical_z
    v = float(intrinsics.cy) + float(intrinsics.fy) * optical_y / optical_z
    return u, v, optical_z


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
    orientation_variant_quaternions=None,
    model_grasp_to_tool_quaternion=None,
):
    variants = list(orientation_variant_quaternions or [])
    if not variants:
        variants = [np.asarray([0.0, 0.0, 0.0, 1.0], dtype=float)]
    ranked = []
    for candidate in candidates:
        camera_candidate = convert_candidate_to_camera_link(candidate, candidate_frame_convention)
        camera_candidate = align_candidate_to_tool_frame(
            camera_candidate,
            model_grasp_to_tool_quaternion,
        )
        for variant_index, correction in enumerate(variants):
            if np.allclose(np.asarray(correction, dtype=float), np.asarray([0.0, 0.0, 0.0, 1.0], dtype=float)):
                variant_candidate = camera_candidate
            else:
                variant_quat = _normalize_quaternion(
                    np.asarray(quaternion_multiply(camera_candidate.quaternion_xyzw, correction), dtype=float)
                )
                variant_candidate = RemoteGraspCandidate(
                    score=float(camera_candidate.score),
                    translation_m=np.asarray(camera_candidate.translation_m, dtype=float),
                    quaternion_xyzw=variant_quat,
                    width_m=float(camera_candidate.width_m),
                    depth_m=getattr(camera_candidate, 'depth_m', None),
                )
            pose = pose_estimator.make_base_pose_from_camera_pose(
                variant_candidate.translation_m,
                variant_candidate.quaternion_xyzw,
                stamp=stamp,
                camera_frame=camera_frame,
            )
            if candidate_filter_fn is not None and not bool(candidate_filter_fn(candidate, variant_candidate, pose)):
                continue
            if bool(reachability_fn(pose)):
                if candidate_rank_fn is None:
                    return variant_candidate, pose
                try:
                    rank = float(candidate_rank_fn(candidate, variant_candidate, pose))
                except Exception:
                    rank = float('inf')
                ranked.append((rank, variant_index, -float(getattr(variant_candidate, 'score', 0.0)), variant_candidate, pose))
    if ranked:
        ranked.sort(key=lambda item: (item[0], item[1], item[2]))
        return ranked[0][3], ranked[0][4]
    return None, None


def make_grasp_plan_pose_array(grasp_pose, stamp, grasp_config):
    plan = make_grasp_sequence_from_grasp_pose(
        grasp_pose,
        pregrasp_distance_m=float(grasp_config.get('pregrasp_distance_m', 0.08)),
        approach_offset_m=float(grasp_config.get('final_approach_offset_m', 0.015)),
        lift_height_m=float(grasp_config.get('lift_height_m', 0.05)),
        tool_approach_axis=str(grasp_config.get('tool_approach_axis', 'x')),
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
        self.grasp_config = dict(gcfg or {})
        self.handeye_parent_frame = str(hcfg.get('parent_frame', 'tool0'))
        self.handeye_camera_frame = str(hcfg.get('camera_frame', cam_cfg.get('frame_id', 'camera_link')))
        self.handeye_translation_xyz = list(hcfg.get('translation_xyz', [0.0, 0.0, 0.0]))
        self.handeye_rotation_xyzw = list(hcfg.get('rotation_xyzw', [0.0, 0.0, 0.0, 1.0]))
        self._cached_tool_from_camera = None
        server_url = rospy.get_param('/grasp_6d/remote/server_url', remote_cfg.get('server_url', 'http://172.23.132.97:8000'))
        timeout_sec = float(rospy.get_param('/grasp_6d/remote/timeout_sec', remote_cfg.get('timeout_sec', 3.0)))
        self.max_candidates = int(rospy.get_param('/grasp_6d/remote/max_candidates', remote_cfg.get('max_candidates', 20)))
        self.auto_request = bool(rospy.get_param('/grasp_6d/remote/auto_request', remote_cfg.get('auto_request', False)))
        self.failure_backoff_sec = max(0.0, float(rospy.get_param('/grasp_6d/remote/failure_backoff_sec', remote_cfg.get('failure_backoff_sec', 8.0))))
        self.candidate_frame_convention = normalize_candidate_frame_convention(
            rospy.get_param('/grasp_6d/remote/candidate_frame_convention', remote_cfg.get('candidate_frame_convention', 'opencv_optical'))
        )
        self.orientation_variant_quaternions = self._parse_orientation_variant_quaternions(
            rospy.get_param('/grasp_6d/remote/orientation_variants_rpy_deg', remote_cfg.get('orientation_variants_rpy_deg', [[0.0, 0.0, 0.0]]))
        )
        self.model_grasp_to_tool_quaternion = self._parse_orientation_variant_quaternions(
            [rospy.get_param(
                '/grasp_6d/remote/model_grasp_to_tool_rpy_deg',
                remote_cfg.get('model_grasp_to_tool_rpy_deg', [0.0, 0.0, 0.0]),
            )]
        )[0]
        self.require_candidate_depth = bool(
            rospy.get_param(
                '/grasp_6d/remote/require_candidate_depth',
                remote_cfg.get('require_candidate_depth', False),
            )
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
        self.camera_visibility_gate_enabled = bool(
            rospy.get_param(
                '/grasp_6d/remote/camera_visibility_gate_enabled',
                remote_cfg.get('camera_visibility_gate_enabled', True),
            )
        )
        self.camera_visibility_diagnostic_enabled = bool(
            rospy.get_param(
                '/grasp_6d/remote/camera_visibility_diagnostic_enabled',
                remote_cfg.get('camera_visibility_diagnostic_enabled', True),
            )
        )
        self.camera_visibility_require_approach = bool(
            rospy.get_param(
                '/grasp_6d/remote/camera_visibility_require_approach',
                remote_cfg.get('camera_visibility_require_approach', True),
            )
        )
        self.camera_visibility_margin_px = max(
            0,
            int(
                rospy.get_param(
                    '/grasp_6d/remote/camera_visibility_margin_px',
                    remote_cfg.get('camera_visibility_margin_px', 36),
                )
            ),
        )
        self.camera_visibility_min_depth_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/camera_visibility_min_depth_m',
                    remote_cfg.get('camera_visibility_min_depth_m', 0.035),
                )
            ),
        )
        self.camera_visibility_max_depth_m = max(
            self.camera_visibility_min_depth_m,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/camera_visibility_max_depth_m',
                    remote_cfg.get('camera_visibility_max_depth_m', 1.20),
                )
            ),
        )
        self.camera_visibility_rank_weight_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/camera_visibility_rank_weight_m',
                    remote_cfg.get('camera_visibility_rank_weight_m', 0.012),
                )
            ),
        )
        self.rank_by_target_distance = bool(
            rospy.get_param(
                '/grasp_6d/remote/rank_by_target_distance',
                remote_cfg.get('rank_by_target_distance', True),
            )
        )
        self.target_position_refine_enabled = bool(
            rospy.get_param(
                '/grasp_6d/remote/target_position_refine_enabled',
                remote_cfg.get('target_position_refine_enabled', False),
            )
        )
        self.target_position_refine_blend = self._clamp01(
            rospy.get_param(
                '/grasp_6d/remote/target_position_refine_blend',
                remote_cfg.get('target_position_refine_blend', 0.0),
            )
        )
        self.target_position_refine_max_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/target_position_refine_max_m',
                    remote_cfg.get('target_position_refine_max_m', 0.04),
                )
            ),
        )
        self.target_position_refine_max_age_sec = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/target_position_refine_max_age_sec',
                    remote_cfg.get('target_position_refine_max_age_sec', 1.0),
                )
            ),
        )
        self.target_position_refine_offset_xyz_m = self._parse_xyz_param(
            rospy.get_param(
                '/grasp_6d/remote/target_position_refine_offset_xyz_m',
                remote_cfg.get('target_position_refine_offset_xyz_m', [0.0, 0.0, 0.0]),
            )
        )
        self.target_cloud_enabled = bool(
            rospy.get_param('/grasp_6d/remote/target_cloud_enabled', remote_cfg.get('target_cloud_enabled', True))
        )
        self.target_cloud_roi_margin_px = int(
            rospy.get_param('/grasp_6d/remote/target_cloud_roi_margin_px', remote_cfg.get('target_cloud_roi_margin_px', 0))
        )
        self.target_cloud_foreground_percentile = self._clamp_range(
            rospy.get_param(
                '/grasp_6d/remote/target_cloud_foreground_percentile',
                remote_cfg.get('target_cloud_foreground_percentile', 35.0),
            ),
            1.0,
            95.0,
        )
        self.target_cloud_depth_window_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/target_cloud_depth_window_m',
                    remote_cfg.get('target_cloud_depth_window_m', 0.055),
                )
            ),
        )
        self.target_cloud_min_points = max(
            1,
            int(rospy.get_param('/grasp_6d/remote/target_cloud_min_points', remote_cfg.get('target_cloud_min_points', 80))),
        )
        self.target_cloud_max_age_sec = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/target_cloud_max_age_sec',
                    remote_cfg.get('target_cloud_max_age_sec', 1.0),
                )
            ),
        )
        self.target_cloud_max_points_for_gate = max(
            1,
            int(
                rospy.get_param(
                    '/grasp_6d/remote/target_cloud_max_points_for_gate',
                    remote_cfg.get('target_cloud_max_points_for_gate', 2500),
                )
            ),
        )
        self.target_cloud_candidate_max_point_distance_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/target_cloud_candidate_max_point_distance_m',
                    remote_cfg.get('target_cloud_candidate_max_point_distance_m', 0.055),
                )
            ),
        )
        self.target_projection_frame_convention = normalize_candidate_frame_convention(
            rospy.get_param(
                '/grasp_6d/remote/target_projection_frame_convention',
                remote_cfg.get('target_projection_frame_convention', 'ros_camera_link'),
            )
        )
        self.latest_target_cloud_base_xyz = None
        self.latest_target_cloud_camera_center = None
        self.latest_target_cloud_camera_points = None
        self.latest_target_cloud_time = None
        self.latest_target_cloud_count = 0
        self.latest_target_cloud_source = 'none'
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
        gripper_cfg = rospy.get_param('/gripper', {})
        twin_cfg = rospy.get_param('/mujoco_digital_twin', {})
        default_max_width = float(
            twin_cfg.get(
                'open_width_m',
                gripper_cfg.get('open_position_m', remote_cfg.get('max_gripper_width_m', 0.05)),
            )
        )
        self.max_gripper_width_m = max(
            0.0,
            float(rospy.get_param('/grasp_6d/remote/max_gripper_width_m', remote_cfg.get('max_gripper_width_m', default_max_width))),
        )
        self.candidate_width_tolerance_m = max(
            0.0,
            float(rospy.get_param('/grasp_6d/remote/candidate_width_tolerance_m', remote_cfg.get('candidate_width_tolerance_m', 0.003))),
        )
        self._target_gate_rejected_count = 0
        self._visibility_gate_rejected_count = 0
        self._width_gate_rejected_count = 0
        self._depth_gate_rejected_count = 0
        self._width_gate_rejected_keys = set()
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
            self._refresh_runtime_params()
            self._cached_tool_from_camera = None
            depth_for_remote, roi_message = self._depth_for_remote(depth, stamp=stamp, frame_id=frame_id)
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
                max_gripper_width_m=self.max_gripper_width_m,
                candidate_width_tolerance_m=self.candidate_width_tolerance_m,
            )
            remote_diagnostics = dict(getattr(self.client, 'last_diagnostics', {}) or {})
            self._position_only_rejected_count = 0
            self._orientation_fallback_rejected_count = 0
            self._target_gate_rejected_count = 0
            self._visibility_gate_rejected_count = 0
            self._width_gate_rejected_count = 0
            self._depth_gate_rejected_count = 0
            self._width_gate_rejected_keys = set()
            selected, grasp_pose = select_first_reachable_candidate(
                candidates,
                self.pose_estimator,
                self._plan_reachable,
                stamp=stamp,
                camera_frame=frame_id or self.pose_estimator.camera_frame,
                candidate_frame_convention=self.candidate_frame_convention,
                candidate_filter_fn=self._candidate_matches_target,
                candidate_rank_fn=self._candidate_rank if self.rank_by_target_distance else None,
                orientation_variant_quaternions=self.orientation_variant_quaternions,
                model_grasp_to_tool_quaternion=self.model_grasp_to_tool_quaternion,
            )
            if selected is None:
                if (
                    not candidates
                    and int(remote_diagnostics.get('width_rejected', 0) or 0) > 0
                    and int(remote_diagnostics.get('after_width', 0) or 0) == 0
                ):
                    message = (
                        'remote 6D WSL filtered all candidates by gripper width '
                        '(raw=%d, after_collision=%d, width>%0.3fm rejected=%d)'
                        % (
                            int(remote_diagnostics.get('raw_candidates', 0) or 0),
                            int(remote_diagnostics.get('after_collision', 0) or 0),
                            float(remote_diagnostics.get('width_limit_m', self.max_gripper_width_m) or self.max_gripper_width_m),
                            int(remote_diagnostics.get('width_rejected', 0) or 0),
                        )
                    )
                elif (
                    self._width_gate_rejected_count > 0
                    or self._depth_gate_rejected_count > 0
                    or self._target_gate_rejected_count > 0
                    or self._visibility_gate_rejected_count > 0
                    or self._position_only_rejected_count > 0
                    or self._orientation_fallback_rejected_count > 0
                ):
                    message = (
                        'remote 6D returned %d candidates, none executable '
                        '(width>%0.3fm: %d, missing-depth: %d, off-target: %d, target-out-of-view: %d, position-only: %d, orientation-fallback: %d)'
                        % (
                            len(candidates),
                            self.max_gripper_width_m,
                            self._width_gate_rejected_count,
                            self._depth_gate_rejected_count,
                            self._target_gate_rejected_count,
                            self._visibility_gate_rejected_count,
                            self._position_only_rejected_count,
                            self._orientation_fallback_rejected_count,
                        )
                    )
                else:
                    message = 'remote 6D returned %d candidates, none reachable' % len(candidates)
                self._publish_error(message)
                return False, message
            final_target_delta = self._candidate_target_distance(None, None, grasp_pose)
            plan_msg = make_grasp_plan_pose_array(grasp_pose, stamp, rospy.get_param('/grasp', {}))
            self.plan_pub.publish(plan_msg)
            selected_target_xyz, target_source = self._target_base_xyz()
            visibility_message = ''
            if bool(getattr(self, 'camera_visibility_diagnostic_enabled', True)) and selected_target_xyz is not None:
                visible, metrics, reason = self._candidate_visibility_metrics(grasp_pose, selected_target_xyz)
                if metrics:
                    stage_text = ','.join(
                        '%s:u%.0f/v%.0f/z%.3f'
                        % (item['stage'], item['u'], item['v'], item['depth_m'])
                        for item in metrics
                    )
                    visibility_message = ' predicted_view=%s(%s)' % ('visible' if visible else 'lost', stage_text)
                elif not visible:
                    visibility_message = ' predicted_view=unknown(%s)' % reason
            depth_text = 'missing' if selected.depth_m is None else '%.3f' % float(selected.depth_m)
            message = 'remote 6D plan ready score=%.3f width=%.3f depth=%s target_delta=%.3fm target=%s strict_orientation=1 tool_aligned=1' % (
                selected.score,
                selected.width_m,
                depth_text,
                final_target_delta,
                target_source,
            ) + visibility_message
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

    def _depth_for_remote(self, depth, stamp=None, frame_id=''):
        if not self.use_perception_roi:
            return depth, ''
        try:
            obj, obj_time = self._latest_object_snapshot()
            masked, roi, valid = self._masked_depth_for_object(depth, obj, obj_time)
            cloud_message = ''
            if bool(getattr(self, 'target_cloud_enabled', True)):
                cloud_message = ' ' + self._update_target_cloud_estimate(depth, obj, obj_time, stamp, frame_id)
            return masked, 'target ROI x=%d y=%d w=%d h=%d foreground_depth=%d%s' % (
                roi[0],
                roi[1],
                roi[2] - roi[0],
                roi[3] - roi[1],
                valid,
                cloud_message,
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

    def _refresh_runtime_params(self):
        remote_cfg = rospy.get_param('/grasp_6d/remote', {})
        self.grasp_config = dict(rospy.get_param('/grasp', getattr(self, 'grasp_config', {})) or {})
        self.candidate_frame_convention = normalize_candidate_frame_convention(
            rospy.get_param(
                '/grasp_6d/remote/candidate_frame_convention',
                remote_cfg.get('candidate_frame_convention', self.candidate_frame_convention),
            )
        )
        self.orientation_variant_quaternions = self._parse_orientation_variant_quaternions(
            rospy.get_param(
                '/grasp_6d/remote/orientation_variants_rpy_deg',
                remote_cfg.get('orientation_variants_rpy_deg', [[0.0, 0.0, 0.0]]),
            )
        )
        self.model_grasp_to_tool_quaternion = self._parse_orientation_variant_quaternions(
            [rospy.get_param(
                '/grasp_6d/remote/model_grasp_to_tool_rpy_deg',
                remote_cfg.get('model_grasp_to_tool_rpy_deg', [0.0, 0.0, 0.0]),
            )]
        )[0]
        self.require_candidate_depth = bool(
            rospy.get_param(
                '/grasp_6d/remote/require_candidate_depth',
                remote_cfg.get('require_candidate_depth', getattr(self, 'require_candidate_depth', False)),
            )
        )
        self.max_candidates = int(
            rospy.get_param('/grasp_6d/remote/max_candidates', remote_cfg.get('max_candidates', self.max_candidates))
        )
        self.allow_position_only_fallback = bool(
            rospy.get_param(
                '/grasp_6d/remote/accept_position_only_fallback',
                remote_cfg.get('accept_position_only_fallback', self.allow_position_only_fallback),
            )
        )
        self.allow_orientation_fallback = bool(
            rospy.get_param(
                '/grasp_6d/remote/accept_orientation_fallback',
                remote_cfg.get('accept_orientation_fallback', self.allow_orientation_fallback),
            )
        )
        self.candidate_target_gate_enabled = bool(
            rospy.get_param(
                '/grasp_6d/remote/candidate_target_gate_enabled',
                remote_cfg.get('candidate_target_gate_enabled', self.candidate_target_gate_enabled),
            )
        )
        self.camera_visibility_gate_enabled = bool(
            rospy.get_param(
                '/grasp_6d/remote/camera_visibility_gate_enabled',
                remote_cfg.get(
                    'camera_visibility_gate_enabled',
                    getattr(self, 'camera_visibility_gate_enabled', True),
                ),
            )
        )
        self.camera_visibility_diagnostic_enabled = bool(
            rospy.get_param(
                '/grasp_6d/remote/camera_visibility_diagnostic_enabled',
                remote_cfg.get(
                    'camera_visibility_diagnostic_enabled',
                    getattr(self, 'camera_visibility_diagnostic_enabled', True),
                ),
            )
        )
        self.camera_visibility_require_approach = bool(
            rospy.get_param(
                '/grasp_6d/remote/camera_visibility_require_approach',
                remote_cfg.get(
                    'camera_visibility_require_approach',
                    getattr(self, 'camera_visibility_require_approach', True),
                ),
            )
        )
        self.camera_visibility_margin_px = max(
            0,
            int(
                rospy.get_param(
                    '/grasp_6d/remote/camera_visibility_margin_px',
                    remote_cfg.get(
                        'camera_visibility_margin_px',
                        getattr(self, 'camera_visibility_margin_px', 36),
                    ),
                )
            ),
        )
        self.camera_visibility_min_depth_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/camera_visibility_min_depth_m',
                    remote_cfg.get(
                        'camera_visibility_min_depth_m',
                        getattr(self, 'camera_visibility_min_depth_m', 0.035),
                    ),
                )
            ),
        )
        self.camera_visibility_max_depth_m = max(
            self.camera_visibility_min_depth_m,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/camera_visibility_max_depth_m',
                    remote_cfg.get(
                        'camera_visibility_max_depth_m',
                        getattr(self, 'camera_visibility_max_depth_m', 1.20),
                    ),
                )
            ),
        )
        self.camera_visibility_rank_weight_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/camera_visibility_rank_weight_m',
                    remote_cfg.get(
                        'camera_visibility_rank_weight_m',
                        getattr(self, 'camera_visibility_rank_weight_m', 0.012),
                    ),
                )
            ),
        )
        self.rank_by_target_distance = bool(
            rospy.get_param(
                '/grasp_6d/remote/rank_by_target_distance',
                remote_cfg.get('rank_by_target_distance', getattr(self, 'rank_by_target_distance', True)),
            )
        )
        self.target_position_refine_enabled = bool(
            rospy.get_param(
                '/grasp_6d/remote/target_position_refine_enabled',
                remote_cfg.get('target_position_refine_enabled', getattr(self, 'target_position_refine_enabled', False)),
            )
        )
        self.target_position_refine_blend = self._clamp01(
            rospy.get_param(
                '/grasp_6d/remote/target_position_refine_blend',
                remote_cfg.get('target_position_refine_blend', getattr(self, 'target_position_refine_blend', 0.0)),
            )
        )
        self.target_position_refine_max_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/target_position_refine_max_m',
                    remote_cfg.get('target_position_refine_max_m', getattr(self, 'target_position_refine_max_m', 0.04)),
                )
            ),
        )
        self.target_position_refine_max_age_sec = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/target_position_refine_max_age_sec',
                    remote_cfg.get(
                        'target_position_refine_max_age_sec',
                        getattr(self, 'target_position_refine_max_age_sec', 1.0),
                    ),
                )
            ),
        )
        self.target_position_refine_offset_xyz_m = self._parse_xyz_param(
            rospy.get_param(
                '/grasp_6d/remote/target_position_refine_offset_xyz_m',
                remote_cfg.get(
                    'target_position_refine_offset_xyz_m',
                    getattr(self, 'target_position_refine_offset_xyz_m', [0.0, 0.0, 0.0]),
                ),
            )
        )
        self.target_cloud_enabled = bool(
            rospy.get_param(
                '/grasp_6d/remote/target_cloud_enabled',
                remote_cfg.get('target_cloud_enabled', getattr(self, 'target_cloud_enabled', True)),
            )
        )
        self.target_cloud_roi_margin_px = int(
            rospy.get_param(
                '/grasp_6d/remote/target_cloud_roi_margin_px',
                remote_cfg.get('target_cloud_roi_margin_px', getattr(self, 'target_cloud_roi_margin_px', 0)),
            )
        )
        self.target_cloud_foreground_percentile = self._clamp_range(
            rospy.get_param(
                '/grasp_6d/remote/target_cloud_foreground_percentile',
                remote_cfg.get(
                    'target_cloud_foreground_percentile',
                    getattr(self, 'target_cloud_foreground_percentile', 35.0),
                ),
            ),
            1.0,
            95.0,
        )
        self.target_cloud_depth_window_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/target_cloud_depth_window_m',
                    remote_cfg.get('target_cloud_depth_window_m', getattr(self, 'target_cloud_depth_window_m', 0.055)),
                )
            ),
        )
        self.target_cloud_min_points = max(
            1,
            int(
                rospy.get_param(
                    '/grasp_6d/remote/target_cloud_min_points',
                    remote_cfg.get('target_cloud_min_points', getattr(self, 'target_cloud_min_points', 80)),
                )
            ),
        )
        self.target_cloud_max_age_sec = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/target_cloud_max_age_sec',
                    remote_cfg.get('target_cloud_max_age_sec', getattr(self, 'target_cloud_max_age_sec', 1.0)),
                )
            ),
        )
        self.target_cloud_max_points_for_gate = max(
            1,
            int(
                rospy.get_param(
                    '/grasp_6d/remote/target_cloud_max_points_for_gate',
                    remote_cfg.get(
                        'target_cloud_max_points_for_gate',
                        getattr(self, 'target_cloud_max_points_for_gate', 2500),
                    ),
                )
            ),
        )
        self.target_cloud_candidate_max_point_distance_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/target_cloud_candidate_max_point_distance_m',
                    remote_cfg.get(
                        'target_cloud_candidate_max_point_distance_m',
                        getattr(self, 'target_cloud_candidate_max_point_distance_m', 0.055),
                    ),
                )
            ),
        )
        self.target_projection_frame_convention = normalize_candidate_frame_convention(
            rospy.get_param(
                '/grasp_6d/remote/target_projection_frame_convention',
                remote_cfg.get(
                    'target_projection_frame_convention',
                    getattr(self, 'target_projection_frame_convention', 'ros_camera_link'),
                ),
            )
        )
        self.candidate_max_target_distance_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/candidate_max_target_distance_m',
                    remote_cfg.get('candidate_max_target_distance_m', self.candidate_max_target_distance_m),
                )
            ),
        )
        self.candidate_min_relative_z_m = float(
            rospy.get_param(
                '/grasp_6d/remote/candidate_min_relative_z_m',
                remote_cfg.get('candidate_min_relative_z_m', self.candidate_min_relative_z_m),
            )
        )
        self.candidate_max_relative_z_m = float(
            rospy.get_param(
                '/grasp_6d/remote/candidate_max_relative_z_m',
                remote_cfg.get('candidate_max_relative_z_m', self.candidate_max_relative_z_m),
            )
        )
        gripper_cfg = rospy.get_param('/gripper', {})
        twin_cfg = rospy.get_param('/mujoco_digital_twin', {})
        default_max_width = float(
            twin_cfg.get(
                'open_width_m',
                gripper_cfg.get('open_position_m', getattr(self, 'max_gripper_width_m', 0.05)),
            )
        )
        self.max_gripper_width_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/max_gripper_width_m',
                    remote_cfg.get('max_gripper_width_m', default_max_width),
                )
            ),
        )
        self.candidate_width_tolerance_m = max(
            0.0,
            float(
                rospy.get_param(
                    '/grasp_6d/remote/candidate_width_tolerance_m',
                    remote_cfg.get('candidate_width_tolerance_m', getattr(self, 'candidate_width_tolerance_m', 0.003)),
                )
            ),
        )

    @staticmethod
    def _parse_orientation_variant_quaternions(value):
        variants = []
        for item in list(value or []):
            if isinstance(item, str):
                parts = [float(part.strip()) for part in item.replace(',', ' ').split() if part.strip()]
            else:
                parts = [float(part) for part in item]
            if len(parts) != 3:
                raise ValueError('orientation variant must be [roll_deg, pitch_deg, yaw_deg]')
            roll, pitch, yaw = [math.radians(part) for part in parts]
            variants.append(_normalize_quaternion(np.asarray(quaternion_from_euler(roll, pitch, yaw), dtype=float)))
        if not variants:
            variants.append(np.asarray([0.0, 0.0, 0.0, 1.0], dtype=float))
        return variants

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
        foreground_mask, valid = self._foreground_mask_for_roi(depth, roi)
        masked = np.zeros_like(depth)
        x0, y0, x1, y1 = roi
        masked_roi = masked[y0:y1, x0:x1]
        depth_roi = np.asarray(depth)[y0:y1, x0:x1]
        masked_roi[foreground_mask] = depth_roi[foreground_mask]
        if valid < self.roi_min_valid_depth_px:
            raise RuntimeError('target foreground has too few valid depth pixels %d < %d' % (valid, self.roi_min_valid_depth_px))
        return masked, roi, valid

    def _foreground_mask_for_roi(self, depth, roi, min_points=None):
        x0, y0, x1, y1 = roi
        depth_roi = np.asarray(depth)[y0:y1, x0:x1]
        if depth_roi.size == 0:
            raise RuntimeError('empty target depth ROI')
        z_m = self._depth_to_meters(depth_roi)
        pcfg = rospy.get_param('/perception', {})
        depth_min = max(0.0, float(pcfg.get('depth_min_m', 0.03)))
        depth_max = max(depth_min, float(pcfg.get('depth_max_m', 2.0)))
        valid = np.isfinite(z_m) & (z_m >= depth_min) & (z_m <= depth_max)
        min_count = max(1, int(min_points if min_points is not None else getattr(self, 'target_cloud_min_points', 80)))
        if int(np.count_nonzero(valid)) < min_count:
            return np.zeros_like(valid, dtype=bool), int(np.count_nonzero(valid))
        values = z_m[valid]
        foreground_z = float(np.percentile(values, float(getattr(self, 'target_cloud_foreground_percentile', 35.0))))
        window = max(0.0, float(getattr(self, 'target_cloud_depth_window_m', 0.055)))
        foreground = valid & (z_m <= foreground_z + window)
        if int(np.count_nonzero(foreground)) < min_count:
            widened = valid & (z_m <= foreground_z + max(window * 2.0, 0.08))
            if int(np.count_nonzero(widened)) >= min_count:
                foreground = widened
            else:
                foreground = valid
        return foreground, int(np.count_nonzero(foreground))

    def _update_target_cloud_estimate(self, depth, obj, obj_time, stamp=None, frame_id=''):
        if obj is None or not bool(getattr(obj, 'detected', False)):
            raise RuntimeError('no detected target object for target cloud')
        if obj_time is None:
            raise RuntimeError('target cloud timestamp unavailable')
        age = (rospy.Time.now() - obj_time).to_sec()
        if age > self.roi_max_age_sec:
            raise RuntimeError('target cloud ROI is stale %.2fs > %.2fs' % (age, self.roi_max_age_sec))
        roi = expanded_bbox_roi(
            np.asarray(depth).shape,
            int(getattr(obj, 'bbox_x', 0)),
            int(getattr(obj, 'bbox_y', 0)),
            int(getattr(obj, 'bbox_width', 0)),
            int(getattr(obj, 'bbox_height', 0)),
            self.target_cloud_roi_margin_px,
        )
        foreground_mask, valid = self._foreground_mask_for_roi(depth, roi, min_points=self.target_cloud_min_points)
        if valid < self.target_cloud_min_points:
            raise RuntimeError('target cloud has too few foreground points %d < %d' % (valid, self.target_cloud_min_points))
        points_camera = self._target_cloud_points_camera(depth, roi, foreground_mask)
        if points_camera.size == 0:
            raise RuntimeError('target cloud projection produced no points')
        center_camera = np.median(points_camera, axis=0).astype(float)
        _pose_cam, pose_base = self.pose_estimator.make_poses(
            center_camera,
            stamp=stamp,
            camera_frame=frame_id or self.pose_estimator.camera_frame,
        )
        position = pose_base.pose.position
        max_points = max(1, int(getattr(self, 'target_cloud_max_points_for_gate', 2500)))
        if len(points_camera) > max_points:
            indices = np.linspace(0, len(points_camera) - 1, max_points).astype(int)
            gate_points = points_camera[indices]
        else:
            gate_points = points_camera
        self.latest_target_cloud_base_xyz = np.asarray([position.x, position.y, position.z], dtype=float)
        self.latest_target_cloud_camera_center = center_camera
        self.latest_target_cloud_camera_points = np.asarray(gate_points, dtype=np.float32)
        self.latest_target_cloud_time = rospy.Time.now()
        self.latest_target_cloud_count = int(len(points_camera))
        self.latest_target_cloud_source = 'roi_depth_foreground'
        return 'target_cloud=%d center_base=(%.3f,%.3f,%.3f)' % (
            int(len(points_camera)),
            float(position.x),
            float(position.y),
            float(position.z),
        )

    def _target_cloud_points_camera(self, depth, roi, foreground_mask):
        x0, y0, x1, y1 = roi
        depth_roi = np.asarray(depth)[y0:y1, x0:x1]
        z_m = self._depth_to_meters(depth_roi)
        ys, xs = np.nonzero(foreground_mask)
        if len(xs) == 0:
            return np.zeros((0, 3), dtype=np.float32)
        intrinsics = self._camera_intrinsics()
        u = xs.astype(np.float32) + float(x0)
        v = ys.astype(np.float32) + float(y0)
        z = z_m[ys, xs].astype(np.float32)
        x_cv = (u - float(intrinsics.cx)) * z / float(intrinsics.fx)
        y_cv = (v - float(intrinsics.cy)) * z / float(intrinsics.fy)
        points_cv = np.stack([x_cv, y_cv, z], axis=1).astype(np.float32)
        return self._project_points_for_camera_frame(points_cv)

    def _depth_to_meters(self, depth_values):
        values = np.asarray(depth_values)
        if np.issubdtype(values.dtype, np.floating):
            return values.astype(np.float32)
        return values.astype(np.float32) * float(self._camera_intrinsics().depth_scale)

    def _candidate_matches_target(self, _candidate, _camera_candidate, grasp_pose):
        depth = getattr(_camera_candidate, 'depth_m', None)
        if bool(getattr(self, 'require_candidate_depth', False)):
            try:
                depth_valid = depth is not None and np.isfinite(float(depth)) and float(depth) > 0.0
            except Exception:
                depth_valid = False
            if not depth_valid:
                self._depth_gate_rejected_count += 1
                rospy.logwarn_throttle(
                    1.0,
                    'remote 6D candidate rejected: WSL response has no valid GraspNet depth_m; sync and restart graspnet_baseline_server.py',
                )
                return False
        width = float(getattr(_camera_candidate, 'width_m', 0.0) or 0.0)
        width_limit = float(getattr(self, 'max_gripper_width_m', 0.0) or 0.0)
        width_tol = float(getattr(self, 'candidate_width_tolerance_m', 0.0) or 0.0)
        if width_limit > 0.0 and width > width_limit + width_tol:
            key = (
                round(float(width), 4),
                tuple(round(float(v), 4) for v in getattr(_camera_candidate, 'translation_m', [])[:3]),
            )
            if key not in self._width_gate_rejected_keys:
                self._width_gate_rejected_keys.add(key)
                self._width_gate_rejected_count += 1
            rospy.logwarn_throttle(
                1.0,
                'remote 6D candidate rejected by width gate: width=%.3fm limit=%.3fm tolerance=%.3fm score=%.3f',
                width,
                width_limit,
                width_tol,
                float(getattr(_camera_candidate, 'score', 0.0) or 0.0),
            )
            return False
        if not getattr(self, 'candidate_target_gate_enabled', True):
            return True
        target_xyz, target_source = self._target_base_xyz()
        if target_xyz is None:
            self._target_gate_rejected_count += 1
            rospy.logwarn_throttle(1.0, 'remote 6D candidate rejected: no locked detection target')
            return False
        try:
            grasp = grasp_pose.pose.position
            dx = float(grasp.x) - float(target_xyz[0])
            dy = float(grasp.y) - float(target_xyz[1])
            dz = float(grasp.z) - float(target_xyz[2])
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        except Exception as exc:
            self._target_gate_rejected_count += 1
            rospy.logwarn_throttle(1.0, 'remote 6D candidate rejected: cannot compare target pose: %s', exc)
            return False
        cloud_distance = self._camera_candidate_cloud_distance(_camera_candidate)
        max_cloud_distance = float(getattr(self, 'target_cloud_candidate_max_point_distance_m', 0.055) or 0.0)
        if np.isfinite(cloud_distance) and max_cloud_distance > 0.0 and cloud_distance > max_cloud_distance:
            self._target_gate_rejected_count += 1
            rospy.logwarn_throttle(
                1.0,
                (
                    'remote 6D candidate rejected by target cloud: '
                    'cloud_dist=%.3fm limit=%.3fm target_source=%s score=%.3f'
                ),
                cloud_distance,
                max_cloud_distance,
                target_source,
                float(getattr(_camera_candidate, 'score', 0.0) or 0.0),
            )
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
                    'delta=(%.3f, %.3f, %.3f) dist=%.3f limits dist<=%.3f z=[%.3f, %.3f] source=%s'
                ),
                float(grasp.x),
                float(grasp.y),
                float(grasp.z),
                float(target_xyz[0]),
                float(target_xyz[1]),
                float(target_xyz[2]),
                dx,
                dy,
                dz,
                distance,
                max_distance,
                min_z,
                max_z,
                target_source,
            )
            return False
        if bool(getattr(self, 'camera_visibility_gate_enabled', False)):
            visible, _metrics, reason = self._candidate_visibility_metrics(grasp_pose, target_xyz)
            if not visible:
                self._visibility_gate_rejected_count += 1
                rospy.logwarn_throttle(
                    1.0,
                    'remote 6D candidate rejected by eye-in-hand visibility gate: %s score=%.3f',
                    reason,
                    float(getattr(_camera_candidate, 'score', 0.0) or 0.0),
                )
                return False
        return True

    def _candidate_rank(self, candidate, camera_candidate, grasp_pose):
        rank = self._candidate_target_distance(candidate, camera_candidate, grasp_pose)
        weight = max(0.0, float(getattr(self, 'camera_visibility_rank_weight_m', 0.0) or 0.0))
        if weight <= 0.0 or not bool(getattr(self, 'camera_visibility_gate_enabled', False)):
            return rank
        target_xyz, _source = self._target_base_xyz()
        if target_xyz is None:
            return float('inf')
        visible, metrics, _reason = self._candidate_visibility_metrics(grasp_pose, target_xyz)
        if not visible or not metrics:
            return float('inf')
        center_cost = max(float(item['center_cost']) for item in metrics)
        return float(rank) + weight * center_cost

    def _candidate_visibility_metrics(self, grasp_pose, target_base_xyz):
        try:
            tool_from_camera = self._tool_from_camera_matrix()
            plan = make_grasp_sequence_from_grasp_pose(
                grasp_pose,
                pregrasp_distance_m=float(self.grasp_config.get('pregrasp_distance_m', 0.08)),
                approach_offset_m=float(self.grasp_config.get('final_approach_offset_m', 0.015)),
                lift_height_m=float(self.grasp_config.get('lift_height_m', 0.05)),
                tool_approach_axis=str(self.grasp_config.get('tool_approach_axis', 'x')),
            )
            stages = [('pregrasp', plan.pregrasp)]
            if bool(getattr(self, 'camera_visibility_require_approach', True)):
                stages.append(('approach', plan.approach))
            intrinsics = self._camera_intrinsics()
            margin = max(0, int(getattr(self, 'camera_visibility_margin_px', 36)))
            min_depth = max(0.0, float(getattr(self, 'camera_visibility_min_depth_m', 0.035)))
            max_depth = max(min_depth, float(getattr(self, 'camera_visibility_max_depth_m', 1.20)))
            x_limit = max(1.0, float(intrinsics.width) * 0.5 - margin)
            y_limit = max(1.0, float(intrinsics.height) * 0.5 - margin)
            metrics = []
            for stage_name, tool_pose in stages:
                u, v, depth = project_base_target_at_tool_pose(
                    tool_pose,
                    target_base_xyz,
                    tool_from_camera,
                    intrinsics,
                )
                center_cost = math.sqrt(
                    ((float(u) - float(intrinsics.cx)) / x_limit) ** 2
                    + ((float(v) - float(intrinsics.cy)) / y_limit) ** 2
                ) if np.isfinite(u) and np.isfinite(v) else float('inf')
                metric = {
                    'stage': stage_name,
                    'u': float(u),
                    'v': float(v),
                    'depth_m': float(depth),
                    'center_cost': float(center_cost),
                }
                metrics.append(metric)
                inside = (
                    np.isfinite(u)
                    and np.isfinite(v)
                    and float(depth) >= min_depth
                    and float(depth) <= max_depth
                    and float(u) >= margin
                    and float(u) < float(intrinsics.width) - margin
                    and float(v) >= margin
                    and float(v) < float(intrinsics.height) - margin
                )
                if not inside:
                    return False, metrics, (
                        '%s predicts uv=(%.1f,%.1f) depth=%.3fm outside margin=%d image=%dx%d'
                        % (
                            stage_name,
                            float(u),
                            float(v),
                            float(depth),
                            margin,
                            int(intrinsics.width),
                            int(intrinsics.height),
                        )
                    )
            return True, metrics, 'visible'
        except Exception as exc:
            return False, [], 'visibility transform failed: %s' % exc

    def _tool_from_camera_matrix(self):
        cached = getattr(self, '_cached_tool_from_camera', None)
        if cached is not None:
            return cached
        matrix = None
        tf_buffer = getattr(self, 'tf_buffer', None)
        if tf_buffer is not None:
            try:
                transform = tf_buffer.lookup_transform(
                    self.handeye_parent_frame,
                    self.handeye_camera_frame,
                    rospy.Time(0),
                    rospy.Duration(0.1),
                )
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                matrix = transform_matrix(
                    [translation.x, translation.y, translation.z],
                    [rotation.x, rotation.y, rotation.z, rotation.w],
                )
            except Exception as exc:
                rospy.logwarn_throttle(
                    2.0,
                    'eye-in-hand visibility TF %s <- %s unavailable; using calibrated fallback: %s',
                    self.handeye_parent_frame,
                    self.handeye_camera_frame,
                    exc,
                )
        if matrix is None:
            matrix = transform_matrix(self.handeye_translation_xyz, self.handeye_rotation_xyzw)
        self._cached_tool_from_camera = matrix
        return matrix

    def _candidate_target_distance(self, _candidate, _camera_candidate, grasp_pose):
        target_xyz, _target_source = self._target_base_xyz()
        if target_xyz is None:
            return float('inf')
        try:
            grasp = grasp_pose.pose.position
            dx = float(grasp.x) - float(target_xyz[0])
            dy = float(grasp.y) - float(target_xyz[1])
            dz = float(grasp.z) - float(target_xyz[2])
            base_distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        except Exception:
            return float('inf')
        cloud_distance = self._camera_candidate_cloud_distance(_camera_candidate)
        if np.isfinite(cloud_distance):
            return float(cloud_distance) + 0.25 * float(base_distance)
        return base_distance

    def _target_base_xyz(self):
        if bool(getattr(self, 'target_cloud_enabled', True)):
            cloud_xyz = getattr(self, 'latest_target_cloud_base_xyz', None)
            cloud_time = getattr(self, 'latest_target_cloud_time', None)
            if cloud_xyz is not None:
                fresh = True
                max_age = float(getattr(self, 'target_cloud_max_age_sec', 1.0) or 0.0)
                if max_age > 0.0 and cloud_time is not None:
                    try:
                        fresh = (rospy.Time.now() - cloud_time).to_sec() <= max_age
                    except Exception:
                        fresh = False
                if fresh:
                    return np.asarray(cloud_xyz, dtype=float), getattr(self, 'latest_target_cloud_source', 'roi_depth_foreground')
        obj, _obj_time = self._latest_object_snapshot()
        if obj is None or not bool(getattr(obj, 'detected', False)):
            return None, 'none'
        try:
            p = obj.pose_base.pose.position
            return np.asarray([float(p.x), float(p.y), float(p.z)], dtype=float), 'perception_object'
        except Exception:
            return None, 'none'

    def _camera_candidate_cloud_distance(self, camera_candidate):
        if camera_candidate is None:
            return float('inf')
        points = getattr(self, 'latest_target_cloud_camera_points', None)
        if points is None:
            return float('inf')
        try:
            cloud = np.asarray(points, dtype=float)
            if cloud.size == 0:
                return float('inf')
            candidate_xyz = np.asarray(camera_candidate.translation_m, dtype=float).reshape(1, 3)
            deltas = cloud - candidate_xyz
            return float(np.min(np.linalg.norm(deltas, axis=1)))
        except Exception:
            return float('inf')

    def _refine_grasp_pose_towards_target(self, grasp_pose):
        if not bool(getattr(self, 'target_position_refine_enabled', False)):
            return grasp_pose, ''
        blend = float(getattr(self, 'target_position_refine_blend', 0.0) or 0.0)
        if blend <= 0.0:
            return grasp_pose, ''
        obj, obj_time = self._latest_object_snapshot()
        if obj is None or not bool(getattr(obj, 'detected', False)):
            return grasp_pose, ''
        if obj_time is not None:
            try:
                age = (rospy.Time.now() - obj_time).to_sec()
            except Exception:
                age = float('inf')
            max_age = float(getattr(self, 'target_position_refine_max_age_sec', 1.0) or 0.0)
            if max_age > 0.0 and age > max_age:
                rospy.logwarn_throttle(
                    1.0,
                    'remote 6D skipped target position refinement: target age %.2fs > %.2fs',
                    age,
                    max_age,
                )
                return grasp_pose, ''
        refined = deepcopy(grasp_pose)
        target = obj.pose_base.pose.position
        current = refined.pose.position
        offset = np.asarray(getattr(self, 'target_position_refine_offset_xyz_m', [0.0, 0.0, 0.0]), dtype=float)
        delta = np.asarray(
            [
                float(target.x) + float(offset[0]) - float(current.x),
                float(target.y) + float(offset[1]) - float(current.y),
                float(target.z) + float(offset[2]) - float(current.z),
            ],
            dtype=float,
        )
        raw_norm = float(np.linalg.norm(delta))
        max_step = float(getattr(self, 'target_position_refine_max_m', 0.04) or 0.0)
        clipped_norm = raw_norm
        if max_step > 0.0 and raw_norm > max_step:
            delta *= max_step / raw_norm
            clipped_norm = max_step
        applied = delta * blend
        current.x = float(current.x) + float(applied[0])
        current.y = float(current.y) + float(applied[1])
        current.z = float(current.z) + float(applied[2])
        message = (
            'refined_to_target blend=%.2f applied=(%.3f,%.3f,%.3f)m raw=%.3fm clipped=%.3fm'
            % (
                blend,
                float(applied[0]),
                float(applied[1]),
                float(applied[2]),
                raw_norm,
                clipped_norm,
            )
        )
        rospy.loginfo('remote 6D %s', message)
        return refined, message

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

    def _project_points_for_camera_frame(self, points_cv):
        points = np.asarray(points_cv, dtype=np.float32)
        if self.target_projection_frame_convention == 'opencv_optical':
            return points
        if self.target_projection_frame_convention == 'ros_camera_link':
            converted = np.empty_like(points)
            converted[:, 0] = points[:, 2]
            converted[:, 1] = -points[:, 0]
            converted[:, 2] = -points[:, 1]
            return converted
        rospy.logwarn_throttle(
            2.0,
            'Unknown target_projection_frame_convention=%s; using OpenCV optical coordinates',
            self.target_projection_frame_convention,
        )
        return points

    @staticmethod
    def _clamp01(value):
        try:
            return min(1.0, max(0.0, float(value)))
        except Exception:
            return 0.0

    @staticmethod
    def _clamp_range(value, low, high):
        try:
            number = float(value)
        except Exception:
            number = float(low)
        return min(float(high), max(float(low), number))

    @staticmethod
    def _parse_xyz_param(value):
        if isinstance(value, str):
            parts = [float(part.strip()) for part in value.replace(',', ' ').split() if part.strip()]
        else:
            parts = [float(part) for part in list(value or [])]
        if len(parts) != 3:
            raise ValueError('xyz parameter must contain exactly 3 values')
        return parts

    def _plan_reachable(self, grasp_pose):
        try:
            strict_pose = (
                not bool(getattr(self, 'allow_position_only_fallback', False))
                and not bool(getattr(self, 'allow_orientation_fallback', False))
            )
            service_name = '/supervisor/check_pose_strict' if strict_pose else '/supervisor/move_to_pose'
            rospy.wait_for_service(service_name, timeout=0.25)
            move_pose = rospy.ServiceProxy(service_name, SetTargetPose)
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
            candidate_fields = set(str(item) for item in (health.get('candidate_fields') or []))
            if bool(getattr(self, 'require_candidate_depth', False)) and 'depth_m' not in candidate_fields:
                message = (
                    'remote 6D server protocol is outdated: depth_m is missing; '
                    'sync tools/graspnet_baseline_server.py to WSL and restart port 8000'
                )
                rospy.logwarn('%s', message)
                self.status_pub.publish(String(message))
                return
            rospy.loginfo(
                'remote 6D server online: backend=%s loaded=%s protocol=%s url=%s',
                health.get('backend', 'unknown'),
                health.get('loaded', 'unknown'),
                health.get('protocol_version', 'unknown'),
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
