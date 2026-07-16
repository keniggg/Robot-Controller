#!/usr/bin/env python3
from copy import deepcopy
from types import SimpleNamespace
import threading

import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
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
from alicia_flexible_grasp.vision.model_selection import resolve_yolo_model_path, select_yolo_model
from alicia_flexible_grasp.vision.yolov8_detector import YOLOv8ObjectDetector
from alicia_flexible_grasp.vision.depth_projector import project_pixel_to_3d
from alicia_flexible_grasp.vision.pose_estimator import PoseEstimator


class LatestFrameBuffer:
    def __init__(self):
        self._lock = threading.Lock()
        self._color = None
        self._color_stamp = None
        self._color_frame_id = ''
        self._color_seq = 0
        self._consumed_color_seq = 0
        self._depth = None

    def update_color(self, color, stamp, frame_id):
        with self._lock:
            self._color = color
            self._color_stamp = stamp
            self._color_frame_id = frame_id
            self._color_seq += 1

    def update_depth(self, depth):
        with self._lock:
            self._depth = depth

    def take_latest(self):
        with self._lock:
            if self._color is None or self._depth is None:
                return None
            if self._color_seq == self._consumed_color_seq:
                return None
            self._consumed_color_seq = self._color_seq
            return SimpleNamespace(
                color=self._color,
                depth=self._depth,
                stamp=self._color_stamp,
                frame_id=self._color_frame_id,
                seq=self._color_seq,
            )


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
        self.detector_task = 'detect'
        self.require_instance_mask = False
        self.enabled = bool(pcfg.get('enabled', True))
        self.detect_hz = max(0.5, float(pcfg.get('detect_hz', 6.0)))
        self.frames = LatestFrameBuffer()
        self.fx = float(cam_cfg.get('fx', 615.0)); self.fy = float(cam_cfg.get('fy', 615.0))
        self.cx = float(cam_cfg.get('cx', cam_cfg.get('width',640)/2.0)); self.cy = float(cam_cfg.get('cy', cam_cfg.get('height',480)/2.0))
        self.depth_scale = float(cam_cfg.get('depth_scale', 0.001))
        self.depth_roi_center_fraction = 0.55
        self.depth_roi_percentile = 50.0
        self.depth_min_m = 0.03
        self.depth_max_m = 2.0
        self.depth_min_valid_px = 24
        self.projection_frame_convention = self._default_projection_frame_convention(hcfg, cam_cfg)
        self._last_depth_source = 'none'
        self._last_depth_valid_count = 0
        self._refresh_depth_params(pcfg)
        self._refresh_projection_params(pcfg)
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
            allow_static_fallback=hcfg.get('allow_static_fallback', True),
        )
        rospy.Subscriber(
            cam_cfg.get('color_topic','/supervisor/camera/color/image_raw'),
            Image,
            self.color_cb,
            queue_size=1,
            buff_size=2**24,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            cam_cfg.get('depth_topic','/supervisor/camera/depth/image_raw'),
            Image,
            self.depth_cb,
            queue_size=1,
            buff_size=2**24,
            tcp_nodelay=True,
        )
        self.pub_obj = rospy.Publisher(pcfg.get('output_object_topic','/perception/object'), ObjectPose, queue_size=10)
        self.pub_mask = rospy.Publisher(
            pcfg.get('output_mask_topic', '/perception/object_mask'),
            Image,
            queue_size=1,
        )
        self.pub_cam = rospy.Publisher(pcfg.get('output_pose_camera_topic','/perception/object_pose_camera'), PoseStamped, queue_size=10)
        self.pub_base = rospy.Publisher(pcfg.get('output_pose_base_topic','/perception/object_pose_base'), PoseStamped, queue_size=10)
        self.pub_detected = rospy.Publisher('/perception/object_detected', Bool, queue_size=10)
        self.pub_raw_detected = rospy.Publisher('/perception/raw_object_detected', Bool, queue_size=10)
        self.detector_status_pub = rospy.Publisher(
            '/perception/detector_status',
            String,
            queue_size=1,
            latch=True,
        )
        self.label = pcfg.get('object_label','target')
        self.stabilizer = DetectionStabilizer(pcfg.get('detection_hold_sec', 0.8))
        self.refresh_detector(force=True)
        self._detect_thread = threading.Thread(target=self.detect_loop, daemon=True)
        self._detect_thread.start()

    def color_cb(self, msg):
        if not self.bridge: return
        color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.frames.update_color(color, msg.header.stamp, msg.header.frame_id)

    def depth_cb(self, msg):
        if not self.bridge: return
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.frames.update_depth(depth)

    def detect_loop(self):
        rate = rospy.Rate(self.detect_hz)
        while not rospy.is_shutdown():
            self._poll_detector_and_process_latest_frame()
            rate.sleep()

    def _poll_detector_and_process_latest_frame(self):
        # Detector configuration must be observed even when the camera stops
        # producing frames, so reload requests can invalidate downstream state.
        self.refresh_detector()
        frame = self.frames.take_latest()
        if frame is None:
            return False
        self.color = frame.color
        self.depth = frame.depth
        self.color_frame_id = frame.frame_id
        self.try_detect(frame.stamp, frame.frame_id)
        return True

    def try_detect(self, stamp, camera_frame=None):
        if self.color is None or self.depth is None:
            return
        self._refresh_camera_params()
        self.refresh_detector()
        segment_mode = PerceptionNode._segment_mode(self)
        if not self.enabled:
            if segment_mode:
                PerceptionNode._publish_target_unavailable(self, stamp, camera_frame)
            else:
                PerceptionNode._publish_target_unavailable(self)
            return
        if self.detector is None:
            rospy.logwarn_throttle(2.0, 'Perception detector unavailable: %s', self.detector_error or 'not initialized')
            if segment_mode:
                PerceptionNode._publish_target_unavailable(self, stamp, camera_frame)
            else:
                PerceptionNode._publish_target_unavailable(self)
            return
        now = rospy.get_time()
        det, mask = self.detector.detect(
            self.color,
            preferred_uv=self.stabilizer.preferred_uv(now),
            max_preferred_distance_px=self.stabilizer.max_jump_px,
        )
        obj = ObjectPose(); obj.header.stamp = stamp; obj.header.frame_id = 'base_link'; obj.label = self.label
        if det is None or (segment_mode and mask is None):
            obj.detected = False
            if segment_mode:
                self._publish_mask(None, stamp, camera_frame)
            self.publish_object(obj); return
        if segment_mode:
            mask_array = np.asarray(mask)
            color_shape = np.asarray(self.color).shape[:2]
            centroid = det.get('mask_centroid')
            if mask_array.shape != color_shape or not np.any(mask_array > 0) or centroid is None:
                obj.detected = False
                self._publish_mask(None, stamp, camera_frame)
                self.publish_object(obj)
                return
            u, v = centroid
        else:
            u, v = det['u'], det['v']
        if v >= self.depth.shape[0] or u >= self.depth.shape[1]:
            obj.detected = False
            if segment_mode:
                self._publish_mask(None, stamp, camera_frame)
            self.publish_object(obj)
            return
        if segment_mode:
            z = self.depth_m_at_mask(mask)
        else:
            z = self.depth_m_at_detection(det, u, v)
        if z <= 0.01:
            obj.detected = False
            if segment_mode:
                self._publish_mask(None, stamp, camera_frame)
            self.publish_object(obj)
            return
        p_cv = project_pixel_to_3d(u, v, z, self.fx, self.fy, self.cx, self.cy)
        p_cam = self._projected_point_for_camera_frame(p_cv)
        try:
            pose_cam, pose_base = self.pose_estimator.make_poses(p_cam, stamp, camera_frame or self.color_frame_id)
        except Exception as exc:
            obj.detected = False
            if segment_mode:
                self._publish_mask(None, stamp, camera_frame)
            rospy.logwarn_throttle(1.0, 'Perception pose transform failed: %s', exc)
            self.publish_object(obj)
            return
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
            'Perception target label=%s frame=%s tf=%s uv=(%d,%d) depth=%.3f[%s n=%d] cam=(%.3f, %.3f, %.3f) base=(%.3f, %.3f, %.3f) conf=%.3f bbox=(%d,%d,%d,%d)',
            obj.label,
            pose_cam.header.frame_id,
            getattr(self.pose_estimator, 'last_transform_source', 'unknown'),
            int(obj.u),
            int(obj.v),
            float(obj.depth_m),
            self._last_depth_source,
            int(self._last_depth_valid_count),
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
        if segment_mode:
            self._publish_mask(mask, stamp, camera_frame)
        self.pub_cam.publish(pose_cam); self.pub_base.publish(pose_base); self.publish_object(obj)

    def publish_object(self, obj):
        # Keep raw visibility separate from the short detection hold used by the
        # GUI. Grasp execution must never mistake a held box for a live view.
        if PerceptionNode._segment_mode(self):
            self.stabilizer.update(obj, rospy.get_time())
            self.pub_raw_detected.publish(Bool(bool(obj.detected)))
            self.pub_obj.publish(obj)
            self.pub_detected.publish(Bool(bool(obj.detected)))
            return
        self.pub_raw_detected.publish(Bool(bool(obj.detected)))
        stable_obj = self.stabilizer.update(obj, rospy.get_time())
        self.pub_obj.publish(stable_obj)
        self.pub_detected.publish(Bool(bool(stable_obj.detected)))

    def _publish_target_unavailable(self, stamp=None, frame_id=None):
        obj = ObjectPose()
        if stamp is not None:
            obj.header.stamp = stamp
            obj.header.frame_id = 'base_link'
        obj.detected = False
        obj.label = str(getattr(self, 'label', '') or '')
        raw_publisher = getattr(self, 'pub_raw_detected', None)
        object_publisher = getattr(self, 'pub_obj', None)
        detected_publisher = getattr(self, 'pub_detected', None)
        if raw_publisher is not None:
            raw_publisher.publish(Bool(False))
        if object_publisher is not None:
            object_publisher.publish(obj)
        if detected_publisher is not None:
            detected_publisher.publish(Bool(False))
        if (
            PerceptionNode._segment_mode(self)
            and getattr(self, 'color', None) is not None
            and getattr(self, 'bridge', None) is not None
            and getattr(self, 'pub_mask', None) is not None
        ):
            PerceptionNode._publish_mask(self, None, obj.header.stamp, frame_id)

    @staticmethod
    def _segment_mode(node):
        return (
            str(getattr(node, 'detector_task', 'detect')).strip().lower() == 'segment'
            or bool(getattr(node, 'require_instance_mask', False))
        )

    def _refresh_camera_params(self):
        cam_cfg = rospy.get_param('/camera', {})
        pcfg = rospy.get_param('/perception', {})
        self.detect_hz = max(0.5, float(pcfg.get('detect_hz', getattr(self, 'detect_hz', 6.0))))
        self.depth_scale = float(cam_cfg.get('depth_scale', self.depth_scale))
        self.fx = float(cam_cfg.get('fx', self.fx))
        self.fy = float(cam_cfg.get('fy', self.fy))
        self.cx = float(cam_cfg.get('cx', self.cx))
        self.cy = float(cam_cfg.get('cy', self.cy))
        self._refresh_depth_params(pcfg)
        self._refresh_projection_params(pcfg)

    def depth_m_at_detection(self, det, u, v):
        self._last_depth_source = 'none'
        self._last_depth_valid_count = 0
        bounds = self._detection_bounds(det, u, v)
        center_bounds = self._center_bounds(bounds, self.depth_roi_center_fraction)
        z = self._depth_stat(center_bounds, 'center_roi')
        if z > 0.0:
            return z
        z = self._depth_stat(bounds, 'bbox_roi')
        if z > 0.0:
            return z
        return self._depth_stat((u, v, u + 1, v + 1), 'center_pixel', min_valid=1)

    def depth_m_at_mask(self, mask):
        self._last_depth_source = 'none'
        self._last_depth_valid_count = 0
        binary = np.asarray(mask) > 0
        if binary.shape != np.asarray(self.depth).shape[:2] or not np.any(binary):
            return 0.0
        values = np.asarray(self.depth)[binary]
        values_m = values.astype(np.float32)
        if not np.issubdtype(values.dtype, np.floating):
            values_m *= float(self.depth_scale)
        valid = values_m[np.isfinite(values_m)]
        valid = valid[(valid >= self.depth_min_m) & (valid <= self.depth_max_m)]
        if valid.size < self.depth_min_valid_px:
            return 0.0
        median = float(np.median(valid))
        mad = float(np.median(np.abs(valid - median)))
        threshold = max(0.002, 3.5 * 1.4826 * mad)
        kept = valid[np.abs(valid - median) <= threshold]
        if kept.size < self.depth_min_valid_px:
            return 0.0
        self._last_depth_source = 'instance_mask'
        self._last_depth_valid_count = int(kept.size)
        return float(np.median(kept))

    def _publish_mask(self, mask, stamp, frame_id):
        shape = np.asarray(self.color).shape[:2]
        output = np.zeros(shape, dtype=np.uint8)
        if mask is not None and np.asarray(mask).shape == shape:
            output[np.asarray(mask) > 0] = 255
        message = self.bridge.cv2_to_imgmsg(output, encoding='mono8')
        message.header.stamp = stamp
        message.header.frame_id = str(frame_id or self.color_frame_id)
        self.pub_mask.publish(message)
        return output

    def _refresh_depth_params(self, pcfg):
        self.depth_roi_center_fraction = self._clamp(
            float(pcfg.get('depth_roi_center_fraction', self.depth_roi_center_fraction)),
            0.10,
            1.0,
        )
        self.depth_roi_percentile = self._clamp(
            float(pcfg.get('depth_roi_percentile', pcfg.get('depth_percentile', self.depth_roi_percentile))),
            5.0,
            95.0,
        )
        self.depth_min_m = max(0.0, float(pcfg.get('depth_min_m', self.depth_min_m)))
        self.depth_max_m = max(self.depth_min_m, float(pcfg.get('depth_max_m', self.depth_max_m)))
        self.depth_min_valid_px = max(1, int(pcfg.get('depth_min_valid_px', self.depth_min_valid_px)))

    def _refresh_projection_params(self, pcfg):
        convention = str(pcfg.get('projection_frame_convention', self.projection_frame_convention)).strip().lower()
        aliases = {
            'ros': 'ros_camera_link',
            'camera_link': 'ros_camera_link',
            'ros_link': 'ros_camera_link',
            'optical': 'opencv_optical',
            'camera_optical': 'opencv_optical',
            'opencv': 'opencv_optical',
        }
        self.projection_frame_convention = aliases.get(convention, convention)

    @staticmethod
    def _default_projection_frame_convention(hcfg, cam_cfg):
        frame = str(hcfg.get('camera_frame', cam_cfg.get('frame_id', 'camera_link'))).lower()
        if frame.endswith('_optical_frame') or frame.endswith('_optical'):
            return 'opencv_optical'
        return 'ros_camera_link'

    def _projected_point_for_camera_frame(self, p_cv):
        if self.projection_frame_convention == 'opencv_optical':
            return list(p_cv)
        if self.projection_frame_convention == 'ros_camera_link':
            x_cv, y_cv, z_cv = [float(v) for v in p_cv[:3]]
            return [z_cv, -x_cv, -y_cv]
        rospy.logwarn_throttle(
            2.0,
            'Unknown projection_frame_convention=%s; using OpenCV optical coordinates',
            self.projection_frame_convention,
        )
        return list(p_cv)

    def _detection_bounds(self, det, u, v):
        x, y, w, h = det.get('bbox', (u, v, 1, 1))
        x0 = int(round(float(x)))
        y0 = int(round(float(y)))
        x1 = x0 + max(1, int(round(float(w))))
        y1 = y0 + max(1, int(round(float(h))))
        return self._clip_bounds((x0, y0, x1, y1))

    def _center_bounds(self, bounds, fraction):
        x0, y0, x1, y1 = bounds
        width = max(1, x1 - x0)
        height = max(1, y1 - y0)
        cw = max(1, int(round(width * float(fraction))))
        ch = max(1, int(round(height * float(fraction))))
        cx = x0 + width // 2
        cy = y0 + height // 2
        return self._clip_bounds((cx - cw // 2, cy - ch // 2, cx - cw // 2 + cw, cy - ch // 2 + ch))

    def _clip_bounds(self, bounds):
        x0, y0, x1, y1 = bounds
        height, width = self.depth.shape[:2]
        x0 = max(0, min(width, int(x0)))
        y0 = max(0, min(height, int(y0)))
        x1 = max(0, min(width, int(x1)))
        y1 = max(0, min(height, int(y1)))
        if x1 <= x0:
            x1 = min(width, x0 + 1)
            x0 = max(0, x1 - 1)
        if y1 <= y0:
            y1 = min(height, y0 + 1)
            y0 = max(0, y1 - 1)
        return x0, y0, x1, y1

    def _depth_stat(self, bounds, source, min_valid=None):
        x0, y0, x1, y1 = self._clip_bounds(bounds)
        roi = np.asarray(self.depth[y0:y1, x0:x1])
        if roi.size == 0:
            return 0.0
        if np.issubdtype(roi.dtype, np.floating):
            roi_m = roi.astype(np.float32, copy=False)
        else:
            roi_m = roi.astype(np.float32, copy=False) * float(self.depth_scale)
        valid = roi_m[np.isfinite(roi_m)]
        valid = valid[(valid >= float(self.depth_min_m)) & (valid <= float(self.depth_max_m))]
        required = self.depth_min_valid_px if min_valid is None else int(min_valid)
        if valid.size < required:
            return 0.0
        if valid.size >= 20:
            lo, hi = np.percentile(valid, [5.0, 95.0])
            trimmed = valid[(valid >= lo) & (valid <= hi)]
            if trimmed.size >= required:
                valid = trimmed
        z = float(np.percentile(valid, self.depth_roi_percentile))
        self._last_depth_source = source
        self._last_depth_valid_count = int(valid.size)
        return z

    @staticmethod
    def _clamp(value, lower, upper):
        return min(float(upper), max(float(lower), float(value)))

    def _publish_detector_status(self, state, choice, detail=''):
        publisher = getattr(self, 'detector_status_pub', None)
        if publisher is None:
            return
        message = '%s:%s' % (str(state), str(choice))
        if detail:
            message += ':' + str(detail)
        publisher.publish(String(data=message))

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
        yolo_choice = str(pcfg.get('yolo_model_choice', 'original'))
        yolo_reload_generation = pcfg.get('yolo_reload_generation', 0)
        yolo_model = str(pcfg.get('yolo_model', 'yolov8n.pt'))
        yolo_target_class = str(pcfg.get('yolo_target_class', label if detector_kind in ('yolo', 'yolov8') else ''))
        yolo_task = 'detect'
        require_instance_mask = False
        resolved_yolo_model = yolo_model
        path_error = None
        if detector_kind in ('yolo', 'yolov8'):
            try:
                selected_model = select_yolo_model(pcfg, yolo_choice, yolo_target_class)
                yolo_target_class = selected_model['target_class']
                yolo_task = selected_model['task']
                require_instance_mask = bool(selected_model['require_instance_mask'])
                resolved_yolo_model = resolve_yolo_model_path(yolo_model)
            except Exception as exc:
                path_error = exc
        yolo_conf = float(pcfg.get('yolo_conf', 0.35))
        yolo_iou = float(pcfg.get('yolo_iou', 0.45))
        yolo_device = str(pcfg.get('yolo_device', 'cpu'))
        yolo_imgsz = int(pcfg.get('yolo_imgsz', 0) or 0)
        signature = (
            enabled, detector_kind, label, lower, upper, tuple(normalized_ranges),
            min_area, shape, hold_seconds, max_jump_px, switch_confirmations,
            resolved_yolo_model, yolo_target_class, yolo_conf, yolo_iou, yolo_device, yolo_imgsz,
            yolo_task, require_instance_mask, yolo_reload_generation,
        )
        if not force and signature == self.detector_signature:
            return
        self.enabled = enabled
        self.label = label
        self.detector_kind = detector_kind
        self.detector_task = yolo_task
        self.require_instance_mask = require_instance_mask
        self.detector_error = ''
        self.detector = None
        self.stabilizer = DetectionStabilizer(hold_seconds, max_jump_px, switch_confirmations)
        PerceptionNode._publish_target_unavailable(self)
        PerceptionNode._publish_detector_status(self, 'loading', yolo_choice)
        try:
            if path_error is not None:
                raise path_error
            if detector_kind in ('yolo', 'yolov8'):
                self.detector = YOLOv8ObjectDetector(
                    model_path=resolved_yolo_model,
                    target_class=yolo_target_class,
                    conf=yolo_conf,
                    iou=yolo_iou,
                    device=yolo_device,
                    imgsz=yolo_imgsz,
                    expected_task=yolo_task,
                    require_instance_mask=require_instance_mask,
                )
                PerceptionNode._publish_detector_status(self, 'ready', yolo_choice, resolved_yolo_model)
            else:
                self.detector_kind = 'simple_hsv'
                self.detector = HSVObjectDetector(lower, upper, min_area, normalized_ranges, shape)
                PerceptionNode._publish_detector_status(self, 'ready', 'simple_hsv')
        except Exception as exc:
            self.detector = None
            detail = str(exc)
            if detail.startswith('YOLO checkpoint task mismatch'):
                detail = 'MODEL_TASK_MISMATCH: ' + detail
            self.detector_error = detail
            PerceptionNode._publish_target_unavailable(self)
            PerceptionNode._publish_detector_status(self, 'error', yolo_choice, self.detector_error)
            rospy.logwarn_throttle(2.0, 'Failed to initialize %s detector: %s', detector_kind, exc)
        self.detector_signature = signature
        rospy.loginfo('Perception detector updated: enabled=%s detector=%s label=%s ranges=%s min_area=%s shape=%s yolo_class=%s',
                      enabled, self.detector_kind, label, normalized_ranges or [(lower, upper)], min_area, shape, yolo_target_class)

if __name__ == '__main__':
    rospy.init_node('perception_node')
    node = PerceptionNode()
    rospy.spin()
