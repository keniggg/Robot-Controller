#!/usr/bin/env python3
import threading

import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image
from std_msgs.msg import String

from alicia_flexible_grasp.grasp.grasp6d_sequence import make_grasp_sequence_from_grasp_pose
from alicia_flexible_grasp.vision.grasp6d_adapter import CameraIntrinsics
from alicia_flexible_grasp.vision.pose_estimator import PoseEstimator
from alicia_flexible_grasp.vision.remote_grasp6d_client import RemoteGrasp6DClient
from alicia_flexible_grasp_supervisor.srv import SetTargetPose, TriggerZero, TriggerZeroResponse


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


def select_first_reachable_candidate(candidates, pose_estimator, reachability_fn, stamp, camera_frame):
    for candidate in candidates:
        pose = pose_estimator.make_base_pose_from_camera_pose(
            candidate.translation_m,
            candidate.quaternion_xyzw,
            stamp=stamp,
            camera_frame=camera_frame,
        )
        if bool(reachability_fn(pose)):
            return candidate, pose
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
        self.last_error = ''
        self._request_lock = threading.Lock()
        self._backoff_until = rospy.Time(0)
        self.plan_pub = rospy.Publisher(rospy.get_param('/grasp/grasp6d_plan_topic', '/grasp_6d/plan'), PoseArray, queue_size=1)
        self.status_pub = rospy.Publisher('/grasp_6d/status', String, queue_size=1, latch=True)

        cam_cfg = rospy.get_param('/camera', {})
        hcfg = rospy.get_param('/handeye', {})
        gcfg = rospy.get_param('/grasp', {})
        remote_cfg = rospy.get_param('/grasp_6d/remote', {})
        server_url = rospy.get_param('/grasp_6d/remote/server_url', remote_cfg.get('server_url', 'http://127.0.0.1:8000'))
        timeout_sec = float(rospy.get_param('/grasp_6d/remote/timeout_sec', remote_cfg.get('timeout_sec', 3.0)))
        self.max_candidates = int(rospy.get_param('/grasp_6d/remote/max_candidates', remote_cfg.get('max_candidates', 20)))
        self.auto_request = bool(rospy.get_param('/grasp_6d/remote/auto_request', remote_cfg.get('auto_request', False)))
        self.failure_backoff_sec = max(0.0, float(rospy.get_param('/grasp_6d/remote/failure_backoff_sec', remote_cfg.get('failure_backoff_sec', 8.0))))
        try:
            self.client = RemoteGrasp6DClient(server_url, timeout_sec=timeout_sec)
        except ValueError as exc:
            rospy.logfatal('invalid remote 6D grasp server URL: %s', exc)
            raise
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
        self.rate_hz = max(0.1, float(rospy.get_param('/grasp_6d/remote/request_hz', remote_cfg.get('request_hz', rospy.get_param('/grasp_6d/plan_hz', 1.0)))))
        rospy.Service('/grasp_6d/request_plan', TriggerZero, self.request_plan_cb)
        self._check_remote_health()
        mode = 'auto %.2f Hz' % self.rate_hz if self.auto_request else 'manual trigger'
        self.status_pub.publish(String('remote 6D grasp waiting for RGB-D: %s (%s)' % (server_url, mode)))

    def color_cb(self, msg):
        self.frames.update_color(self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'), msg.header.stamp, msg.header.frame_id)

    def depth_cb(self, msg):
        self.frames.update_depth(self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'))

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
            self.status_pub.publish(String('remote 6D requesting candidates...'))
            candidates = self.client.predict(
                color,
                depth,
                self._camera_intrinsics(),
                frame_id=frame_id or self.pose_estimator.camera_frame,
                stamp_sec=stamp.to_sec() if stamp is not None else 0.0,
                max_candidates=self.max_candidates,
            )
            selected, grasp_pose = select_first_reachable_candidate(
                candidates,
                self.pose_estimator,
                self._plan_reachable,
                stamp=stamp,
                camera_frame=frame_id or self.pose_estimator.camera_frame,
            )
            if selected is None:
                message = 'remote 6D returned %d candidates, none reachable' % len(candidates)
                self._publish_error(message)
                return False, message
            plan_msg = make_grasp_plan_pose_array(grasp_pose, stamp, rospy.get_param('/grasp', {}))
            self.plan_pub.publish(plan_msg)
            message = 'remote 6D plan ready score=%.3f width=%.3f' % (selected.score, selected.width_m)
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
            return bool(response.success)
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
