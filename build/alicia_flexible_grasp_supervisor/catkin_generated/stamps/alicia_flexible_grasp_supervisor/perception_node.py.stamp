#!/usr/bin/env python3
from copy import deepcopy

import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from alicia_flexible_grasp_supervisor.msg import ObjectPose
try:
    from cv_bridge import CvBridge
except Exception:
    CvBridge = None
try:
    import tf2_ros
except Exception:
    tf2_ros = None
from alicia_flexible_grasp.vision.object_detector import HSVObjectDetector
from alicia_flexible_grasp.vision.yolov8_detector import YOLOv8ObjectDetector
from alicia_flexible_grasp.vision.depth_projector import project_pixel_to_3d
from alicia_flexible_grasp.vision.pose_estimator import PoseEstimator


class DetectionStabilizer:
    def __init__(self, hold_seconds=0.8, max_jump_px=90.0, switch_confirmations=3):
        self.hold_seconds = max(0.0, float(hold_seconds))
        self.max_jump_px = max(0.0, float(max_jump_px))
        self.switch_confirmations = max(1, int(switch_confirmations))
        self.last_detection = None
        self.last_time = None
        self.pending_detection = None
        self.pending_count = 0

    def reset(self):
        self.last_detection = None
        self.last_time = None
        self.pending_detection = None
        self.pending_count = 0

    def update(self, obj, now):
        now = float(now)
        if getattr(obj, 'detected', False):
            if self._should_hold_last_for_jump(obj, now):
                return self._handle_jump_candidate(obj, now)
            return self._accept(obj, now)
        if self.last_detection is None or self.last_time is None:
            return obj
        if now - self.last_time > self.hold_seconds:
            return obj
        return self._held_with_header(obj)

    def preferred_uv(self, now):
        if self.last_detection is None or self.last_time is None:
            return None
        if float(now) - self.last_time > self.hold_seconds:
            return None
        return self._center(self.last_detection)

    def _accept(self, obj, now):
        self.last_detection = deepcopy(obj)
        self.last_time = float(now)
        self.pending_detection = None
        self.pending_count = 0
        return obj

    def _held_with_header(self, obj):
        held = deepcopy(self.last_detection)
        if hasattr(obj, 'header') and hasattr(held, 'header'):
            held.header = obj.header
        return held

    def _should_hold_last_for_jump(self, obj, now):
        if self.max_jump_px <= 0.0 or self.last_detection is None or self.last_time is None:
            return False
        if now - self.last_time > self.hold_seconds:
            return False
        old_center = self._center(self.last_detection)
        new_center = self._center(obj)
        if old_center is None or new_center is None:
            return False
        distance = float(np.hypot(new_center[0] - old_center[0], new_center[1] - old_center[1]))
        return distance > self.max_jump_px

    def _handle_jump_candidate(self, obj, now):
        if self.pending_detection is None or not self._same_pending_region(obj):
            self.pending_detection = deepcopy(obj)
            self.pending_count = 1
        else:
            self.pending_detection = deepcopy(obj)
            self.pending_count += 1
        if self.pending_count >= self.switch_confirmations:
            return self._accept(obj, now)
        return self._held_with_header(obj)

    def _same_pending_region(self, obj):
        old_center = self._center(self.pending_detection)
        new_center = self._center(obj)
        if old_center is None or new_center is None:
            return False
        threshold = max(12.0, self.max_jump_px * 0.5)
        distance = float(np.hypot(new_center[0] - old_center[0], new_center[1] - old_center[1]))
        return distance <= threshold

    @staticmethod
    def _center(obj):
        if obj is None:
            return None
        try:
            return float(obj.u), float(obj.v)
        except Exception:
            pass
        try:
            width = float(getattr(obj, 'bbox_width', 0))
            height = float(getattr(obj, 'bbox_height', 0))
            if width <= 0.0 or height <= 0.0:
                return None
            return (
                float(getattr(obj, 'bbox_x', 0)) + width * 0.5,
                float(getattr(obj, 'bbox_y', 0)) + height * 0.5,
            )
        except Exception:
            return None


class PerceptionNode:
    def __init__(self):
        cam_cfg = rospy.get_param('/camera', {})
        pcfg = rospy.get_param('/perception', {})
        hcfg = rospy.get_param('/handeye', {})
        gcfg = rospy.get_param('/grasp', {})
        self.bridge = CvBridge() if CvBridge else None
        self.color = None
        self.color_frame_id = ''
        self.depth = None
        self.detector = None
        self.detector_kind = 'simple_hsv'
        self.detector_error = ''
        self.detector_signature = None
        self.enabled = bool(pcfg.get('enabled', True))
        self.fx = float(cam_cfg.get('fx', 615.0)); self.fy = float(cam_cfg.get('fy', 615.0))
        self.cx = float(cam_cfg.get('cx', cam_cfg.get('width',640)/2.0)); self.cy = float(cam_cfg.get('cy', cam_cfg.get('height',480)/2.0))
        self.depth_scale = float(cam_cfg.get('depth_scale', 0.001))
        self.tf_buffer = None
        self.tf_listener = None
        if bool(hcfg.get('use_tf', True)) and tf2_ros is not None:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        elif bool(hcfg.get('use_tf', True)):
            rospy.logwarn('tf2_ros is unavailable; perception will use configured static handeye transform')
        self.pose_estimator = PoseEstimator(
            hcfg.get('camera_frame','camera_color_optical_frame'),
            hcfg.get('base_frame','base_link'),
            hcfg.get('translation_xyz',[0,0,0]),
            hcfg.get('rotation_xyzw',[0,0,0,1]),
            gcfg.get('default_orientation_xyzw',[0,0.7071,0,0.7071]),
            tf_buffer=self.tf_buffer,
            tf_timeout_sec=hcfg.get('tf_timeout_sec', 0.2),
            tf_lookup_latest=hcfg.get('tf_lookup_latest', True),
        )
        rospy.Subscriber(cam_cfg.get('color_topic','/supervisor/camera/color/image_raw'), Image, self.color_cb, queue_size=1)
        rospy.Subscriber(cam_cfg.get('depth_topic','/supervisor/camera/depth/image_raw'), Image, self.depth_cb, queue_size=1)
        self.pub_obj = rospy.Publisher(pcfg.get('output_object_topic','/perception/object'), ObjectPose, queue_size=10)
        self.pub_cam = rospy.Publisher(pcfg.get('output_pose_camera_topic','/perception/object_pose_camera'), PoseStamped, queue_size=10)
        self.pub_base = rospy.Publisher(pcfg.get('output_pose_base_topic','/perception/object_pose_base'), PoseStamped, queue_size=10)
        self.pub_detected = rospy.Publisher('/perception/object_detected', Bool, queue_size=10)
        self.label = pcfg.get('object_label','target')
        self.stabilizer = DetectionStabilizer(pcfg.get('detection_hold_sec', 0.8))
        self.refresh_detector(force=True)

    def color_cb(self, msg):
        if not self.bridge: return
        self.color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.color_frame_id = msg.header.frame_id
        self.try_detect(msg.header.stamp, msg.header.frame_id)

    def depth_cb(self, msg):
        if not self.bridge: return
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def try_detect(self, stamp, camera_frame=None):
        if self.color is None or self.depth is None:
            return
        self._refresh_camera_params()
        self.refresh_detector()
        if not self.enabled:
            self.pub_detected.publish(Bool(False))
            return
        if self.detector is None:
            rospy.logwarn_throttle(2.0, 'Perception detector unavailable: %s', self.detector_error or 'not initialized')
            self.pub_detected.publish(Bool(False))
            return
        now = rospy.get_time()
        det, mask = self.detector.detect(
            self.color,
            preferred_uv=self.stabilizer.preferred_uv(now),
            max_preferred_distance_px=self.stabilizer.max_jump_px,
        )
        obj = ObjectPose(); obj.header.stamp = stamp; obj.header.frame_id = 'base_link'; obj.label = self.label
        if det is None:
            obj.detected = False
            self.publish_object(obj); return
        u, v = det['u'], det['v']
        if v >= self.depth.shape[0] or u >= self.depth.shape[1]:
            obj.detected = False
            self.publish_object(obj)
            return
        z = self.depth_m_at_detection(det, u, v)
        if z <= 0.01:
            obj.detected = False
            self.publish_object(obj)
            return
        p_cam = project_pixel_to_3d(u, v, z, self.fx, self.fy, self.cx, self.cy)
        pose_cam, pose_base = self.pose_estimator.make_poses(p_cam, stamp, camera_frame or self.color_frame_id)
        obj.detected = True
        obj.label = str(det.get('label', self.label) or self.label)
        obj.confidence = float(det.get('confidence', 1.0)); obj.u = u; obj.v = v; obj.depth_m = z
        x, y, w, h = det.get('bbox', (0, 0, 0, 0))
        obj.bbox_x = int(max(0, x)); obj.bbox_y = int(max(0, y))
        obj.bbox_width = int(max(0, w)); obj.bbox_height = int(max(0, h))
        obj.pose_camera = pose_cam; obj.pose_base = pose_base
        bp = pose_base.pose.position
        rospy.loginfo_throttle(
            1.0,
            'Perception target label=%s frame=%s uv=(%d,%d) depth=%.3f cam=(%.3f, %.3f, %.3f) base=(%.3f, %.3f, %.3f) conf=%.3f bbox=(%d,%d,%d,%d)',
            obj.label,
            pose_cam.header.frame_id,
            int(obj.u),
            int(obj.v),
            float(obj.depth_m),
            float(p_cam[0]),
            float(p_cam[1]),
            float(p_cam[2]),
            float(bp.x),
            float(bp.y),
            float(bp.z),
            float(obj.confidence),
            int(obj.bbox_x),
            int(obj.bbox_y),
            int(obj.bbox_width),
            int(obj.bbox_height),
        )
        self.pub_cam.publish(pose_cam); self.pub_base.publish(pose_base); self.publish_object(obj)

    def publish_object(self, obj):
        stable_obj = self.stabilizer.update(obj, rospy.get_time())
        self.pub_obj.publish(stable_obj)
        self.pub_detected.publish(Bool(bool(stable_obj.detected)))

    def _refresh_camera_params(self):
        cam_cfg = rospy.get_param('/camera', {})
        self.depth_scale = float(cam_cfg.get('depth_scale', self.depth_scale))
        self.fx = float(cam_cfg.get('fx', self.fx))
        self.fy = float(cam_cfg.get('fy', self.fy))
        self.cx = float(cam_cfg.get('cx', self.cx))
        self.cy = float(cam_cfg.get('cy', self.cy))

    def depth_m_at_detection(self, det, u, v):
        z = float(self.depth[v, u]) * self.depth_scale
        if z > 0.01:
            return z
        x, y, w, h = det.get('bbox', (u, v, 1, 1))
        x0 = max(0, int(x)); y0 = max(0, int(y))
        x1 = min(self.depth.shape[1], x0 + max(1, int(w)))
        y1 = min(self.depth.shape[0], y0 + max(1, int(h)))
        roi = np.asarray(self.depth[y0:y1, x0:x1], dtype=np.float32) * self.depth_scale
        valid = roi[np.isfinite(roi) & (roi > 0.01)]
        if valid.size == 0:
            return 0.0
        return float(np.median(valid))

    def refresh_detector(self, force=False):
        pcfg = rospy.get_param('/perception', {})
        enabled = bool(pcfg.get('enabled', True))
        detector_kind = str(pcfg.get('detector', 'simple_hsv')).lower()
        label = pcfg.get('object_label', 'target')
        lower = tuple(int(v) for v in pcfg.get('hsv_lower', [35, 40, 40])[:3])
        upper = tuple(int(v) for v in pcfg.get('hsv_upper', [85, 255, 255])[:3])
        hsv_ranges = pcfg.get('hsv_ranges', [])
        normalized_ranges = []
        for item in hsv_ranges:
            try:
                low, high = item
                normalized_ranges.append((
                    tuple(int(v) for v in low[:3]),
                    tuple(int(v) for v in high[:3]),
                ))
            except Exception:
                rospy.logwarn_throttle(2.0, 'Ignoring invalid hsv_ranges item: %s', item)
        min_area = int(pcfg.get('min_area', 300))
        shape = str(pcfg.get('shape', 'any')).lower()
        hold_seconds = float(pcfg.get('detection_hold_sec', self.stabilizer.hold_seconds))
        max_jump_px = float(pcfg.get('tracking_max_jump_px', self.stabilizer.max_jump_px))
        switch_confirmations = int(pcfg.get('tracking_switch_confirmations', self.stabilizer.switch_confirmations))
        yolo_model = str(pcfg.get('yolo_model', 'yolov8n.pt'))
        yolo_target_class = str(pcfg.get('yolo_target_class', label if detector_kind in ('yolo', 'yolov8') else ''))
        yolo_conf = float(pcfg.get('yolo_conf', 0.35))
        yolo_iou = float(pcfg.get('yolo_iou', 0.45))
        yolo_device = str(pcfg.get('yolo_device', 'cpu'))
        signature = (
            enabled, detector_kind, label, lower, upper, tuple(normalized_ranges),
            min_area, shape, hold_seconds, max_jump_px, switch_confirmations,
            yolo_model, yolo_target_class, yolo_conf, yolo_iou, yolo_device,
        )
        if not force and signature == self.detector_signature:
            return
        self.enabled = enabled
        self.label = label
        self.detector_kind = detector_kind
        self.detector_error = ''
        self.stabilizer = DetectionStabilizer(hold_seconds, max_jump_px, switch_confirmations)
        try:
            if detector_kind in ('yolo', 'yolov8'):
                self.detector = YOLOv8ObjectDetector(
                    model_path=yolo_model,
                    target_class=yolo_target_class,
                    conf=yolo_conf,
                    iou=yolo_iou,
                    device=yolo_device,
                )
            else:
                self.detector_kind = 'simple_hsv'
                self.detector = HSVObjectDetector(lower, upper, min_area, normalized_ranges, shape)
        except Exception as exc:
            self.detector = None
            self.detector_error = str(exc)
            rospy.logwarn_throttle(2.0, 'Failed to initialize %s detector: %s', detector_kind, exc)
        self.detector_signature = signature
        rospy.loginfo('Perception detector updated: enabled=%s detector=%s label=%s ranges=%s min_area=%s shape=%s yolo_class=%s',
                      enabled, self.detector_kind, label, normalized_ranges or [(lower, upper)], min_area, shape, yolo_target_class)

if __name__ == '__main__':
    rospy.init_node('perception_node')
    node = PerceptionNode()
    rospy.spin()
