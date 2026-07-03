#!/usr/bin/env python3
import threading

import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image
from std_msgs.msg import String

from alicia_flexible_grasp.grasp.grasp6d_candidate_selector import select_best_grasp6d_candidate
from alicia_flexible_grasp.grasp.grasp6d_sequence import make_grasp_sequence_from_grasp_pose
from alicia_flexible_grasp.vision.grasp6d_adapter import (
    AliciaGrasp6DBackend,
    CameraIntrinsics,
    Grasp6DBackendUnavailable,
    build_graspnet_input_from_rgbd,
    check_grasp6d_dependencies,
    inspect_grasp6d_runtime,
)
from alicia_flexible_grasp.vision.pose_estimator import PoseEstimator
from alicia_flexible_grasp_supervisor.msg import TactileState
from alicia_flexible_grasp_supervisor.srv import SetTargetPose


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
            if self.color is None or self.depth is None:
                return None
            if self.color_seq == self.consumed_seq:
                return None
            self.consumed_seq = self.color_seq
            return self.color.copy(), self.depth.copy(), self.stamp, self.frame_id


class Grasp6DNode:
    def __init__(self):
        self.enabled = bool(rospy.get_param('/grasp_6d/enabled', True))
        self.bridge = CvBridge()
        self.frames = LatestRgbdBuffer()
        self.latest_tactile = None
        self.backend = None
        self.last_backend_error = ''
        self.plan_pub = rospy.Publisher(rospy.get_param('/grasp/grasp6d_plan_topic', '/grasp_6d/plan'), PoseArray, queue_size=1)
        self.status_pub = rospy.Publisher('/grasp_6d/status', String, queue_size=1, latch=True)
        cam_cfg = rospy.get_param('/camera', {})
        hcfg = rospy.get_param('/handeye', {})
        gcfg = rospy.get_param('/grasp', {})
        self.pose_estimator = PoseEstimator(
            hcfg.get('camera_frame', cam_cfg.get('frame_id', 'camera_link')),
            hcfg.get('base_frame', 'base_link'),
            hcfg.get('translation_xyz', [0.0, 0.0, 0.0]),
            hcfg.get('rotation_xyzw', [0.0, 0.0, 0.0, 1.0]),
            gcfg.get('default_orientation_xyzw', [0.0, 0.7071, 0.0, 0.7071]),
            tf_buffer=None,
            allow_static_fallback=True,
        )
        rospy.Subscriber(cam_cfg.get('color_topic', '/supervisor/camera/color/image_raw'), Image, self.color_cb, queue_size=1, buff_size=2**24, tcp_nodelay=True)
        rospy.Subscriber(cam_cfg.get('depth_topic', '/supervisor/camera/depth/image_raw'), Image, self.depth_cb, queue_size=1, buff_size=2**24, tcp_nodelay=True)
        rospy.Subscriber('/tactile/state', TactileState, self.tactile_cb, queue_size=1)
        self.rate_hz = max(0.1, float(rospy.get_param('/grasp_6d/plan_hz', 1.0)))
        self._publish_startup_status()

    def color_cb(self, msg):
        self.frames.update_color(self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'), msg.header.stamp, msg.header.frame_id)

    def depth_cb(self, msg):
        self.frames.update_depth(self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'))

    def tactile_cb(self, msg):
        self.latest_tactile = msg

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.enabled:
                self._process_latest_frame()
            rate.sleep()

    def _process_latest_frame(self):
        frame = self.frames.take_latest()
        if frame is None:
            return
        color, depth, stamp, frame_id = frame
        backend = self._ensure_backend()
        if backend is None:
            return
        try:
            grasp_input = build_graspnet_input_from_rgbd(
                color,
                depth,
                self._camera_intrinsics(),
                workspace_mask=None,
                num_points=int(rospy.get_param('/grasp_6d/num_points', 50000)),
                voxel_size=float(rospy.get_param('/grasp_6d/voxel_size', 0.005)),
            )
            candidates = backend.predict_candidates(grasp_input)
            base_candidates = [self._candidate_with_base_pose(candidate, stamp, frame_id) for candidate in candidates]
            for candidate in base_candidates:
                candidate.reachable = self._plan_reachable(candidate.pose_base)
                candidate.tactile_score = self._tactile_safety_score()
            selected = select_best_grasp6d_candidate(
                base_candidates,
                tactile_weight=float(rospy.get_param('/grasp_6d/tactile_weight', 0.2)),
            )
            if selected is None:
                self.status_pub.publish(String('no reachable collision-free 6D grasp candidate'))
                return
            self._publish_plan(selected.pose_base, stamp)
            self.status_pub.publish(String('selected 6D grasp score=%.3f width=%.3f' % (selected.score, selected.width_m or 0.0)))
        except Exception as exc:
            self.status_pub.publish(String('6D grasp planning failed: %s' % exc))
            rospy.logwarn_throttle(2.0, '6D grasp planning failed: %s', exc)

    def _ensure_backend(self):
        if self.backend is not None:
            return self.backend
        status = check_grasp6d_dependencies()
        runtime = inspect_grasp6d_runtime(
            root=rospy.get_param('/grasp_6d/root', ''),
            checkpoint_path=rospy.get_param('/grasp_6d/checkpoint_path', ''),
        )
        if not status.available or not runtime.ready:
            self._publish_backend_error(runtime.message)
            return None
        try:
            self.backend = AliciaGrasp6DBackend(
                root=rospy.get_param('/grasp_6d/root', ''),
                checkpoint_path=rospy.get_param('/grasp_6d/checkpoint_path', ''),
                seed_feat_dim=int(rospy.get_param('/grasp_6d/seed_feat_dim', 512)),
                collision_thresh=float(rospy.get_param('/grasp_6d/collision_thresh', 0.05)),
                collision_voxel_size=float(rospy.get_param('/grasp_6d/collision_voxel_size', 0.01)),
                device=rospy.get_param('/grasp_6d/device', 'cpu'),
            ).load()
            self.status_pub.publish(String('6D grasp backend ready: alicia_d_grasp_6d'))
            return self.backend
        except Grasp6DBackendUnavailable as exc:
            self._publish_backend_error(str(exc))
        except Exception as exc:
            self._publish_backend_error('6D backend load failed: %s' % exc)
        return None

    def _publish_backend_error(self, message):
        if message != self.last_backend_error:
            self.last_backend_error = message
            rospy.logwarn('%s', message)
            self.status_pub.publish(String(message))
        else:
            rospy.logwarn_throttle(5.0, '%s', message)

    def _publish_startup_status(self):
        if not self.enabled:
            self.status_pub.publish(String('6D grasp disabled'))
            return
        report = inspect_grasp6d_runtime(
            root=rospy.get_param('/grasp_6d/root', ''),
            checkpoint_path=rospy.get_param('/grasp_6d/checkpoint_path', ''),
        )
        self.status_pub.publish(String('6D grasp waiting for RGB-D' if report.ready else report.message))

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

    def _candidate_with_base_pose(self, candidate, stamp, camera_frame):
        if candidate.pose_camera is None:
            return candidate
        xyz, quat = candidate.pose_camera
        candidate.pose_base = self.pose_estimator.make_base_pose_from_camera_pose(xyz, quat, stamp=stamp, camera_frame=camera_frame)
        return candidate

    def _plan_reachable(self, grasp_pose):
        if grasp_pose is None:
            return False
        try:
            rospy.wait_for_service('/supervisor/move_to_pose', timeout=0.25)
            move_pose = rospy.ServiceProxy('/supervisor/move_to_pose', SetTargetPose)
            response = move_pose(grasp_pose, False)
            return bool(response.success)
        except Exception:
            return False

    def _tactile_safety_score(self):
        msg = self.latest_tactile
        if msg is None or not bool(getattr(msg, 'valid', False)):
            return 0.7
        max_force = max(1.0, float(rospy.get_param('/force/max_force_mn', 4000.0)))
        force = max(0.0, float(getattr(msg, 'total_grip_force_mn', 0.0)))
        return max(0.0, min(1.0, 1.0 - force / max_force))

    def _publish_plan(self, grasp_pose, stamp):
        gcfg = rospy.get_param('/grasp', {})
        plan = make_grasp_sequence_from_grasp_pose(
            grasp_pose,
            pregrasp_distance_m=float(gcfg.get('pregrasp_distance_m', 0.08)),
            approach_offset_m=float(gcfg.get('final_approach_offset_m', 0.015)),
            lift_height_m=float(gcfg.get('lift_height_m', 0.05)),
        )
        msg = PoseArray()
        msg.header.frame_id = grasp_pose.header.frame_id
        msg.header.stamp = stamp if stamp is not None else rospy.Time.now()
        msg.poses = [plan.pregrasp.pose, plan.approach.pose, plan.grasp.pose, plan.lift.pose]
        self.plan_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('grasp6d_node')
    Grasp6DNode().spin()
