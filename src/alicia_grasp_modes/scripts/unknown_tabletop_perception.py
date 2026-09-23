#!/usr/bin/env python3
"""Independent exact-RGB-D tabletop perception. This node has no arm controls."""
import json
import threading
import time
from dataclasses import fields

import message_filters
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from alicia_flexible_grasp_supervisor.msg import ObjectPose
from alicia_grasp_modes.tabletop import Config, TargetTracker, segment_with_support_fallback
from alicia_grasp_modes.rgb_instance import refine_candidate
from alicia_grasp_modes.acquisition import InitialAcquisition


def transform_matrix(message):
    t, q = message.transform.translation, message.transform.rotation
    quaternion = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = float(quaternion @ quaternion)
    if not np.isfinite(norm) or abs(norm - 1.0) > 0.01:
        raise ValueError('invalid_exact_tf_quaternion')
    x, y, z, w = quaternion / np.sqrt(norm)
    matrix = np.eye(4)
    matrix[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    matrix[:3, 3] = [t.x, t.y, t.z]
    return matrix


class UnknownTabletopPerception:
    def __init__(self):
        cfg = rospy.get_param('/unknown_perception', {})
        camera = rospy.get_param('/camera', {})
        self.config = Config(**{f.name: cfg[f.name] for f in fields(Config) if f.name in cfg})
        self.tracker = TargetTracker(self.config)
        self.acquisition = InitialAcquisition(self.config)
        self.base_frame = str(cfg.get('base_frame', 'base_link'))
        self.camera_frame = str(cfg.get('camera_frame', camera.get('frame_id', 'camera_link')))
        self.max_age = float(cfg.get('max_frame_age_sec', 0.8))
        self.tf_wait = float(cfg.get('tf_wait_sec', 0.15))
        self.minimum_quality = float(cfg.get('minimum_geometry_quality', 0.5))
        self.rgb_instance_refinement = bool(cfg.get('rgb_instance_refinement', True))
        self.bridge = CvBridge()
        self.lock = threading.RLock()
        # Image ingress must not wait for GrabCut, geometry or ROS publishing.
        # Keep only the newest exact pair while a previous pair is processed.
        self.input_lock = threading.RLock()
        self.mode, self.generation, self.selection_ns = 'carton', -1, 0
        self.latest = None
        self.last_consumed_ns = 0
        self.anchor_source_stamp_ns = 0
        self.last_receive_wall = 0.0
        self.last_shape = (int(camera.get('height', 480)), int(camera.get('width', 640)))
        self.buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.listener = tf2_ros.TransformListener(self.buffer)
        prefix = '/perception/unknown/'
        self.pubs = {
            'object': rospy.Publisher(prefix+'object', ObjectPose, queue_size=2),
            'object_mask': rospy.Publisher(prefix+'object_mask', Image, queue_size=2),
            'object_pose_camera': rospy.Publisher(prefix+'object_pose_camera', PoseStamped, queue_size=2),
            'object_pose_base': rospy.Publisher(prefix+'object_pose_base', PoseStamped, queue_size=2),
            'object_detected': rospy.Publisher(prefix+'object_detected', Bool, queue_size=2),
            'raw_object_detected': rospy.Publisher(prefix+'raw_object_detected', Bool, queue_size=2),
            'detector_status': rospy.Publisher(prefix+'detector_status', String, queue_size=2, latch=True),
        }
        self.selection_sub = rospy.Subscriber('/grasp_mode/selection', String, self.selection, queue_size=1)
        self.color_sub = message_filters.Subscriber(camera.get('color_topic', '/supervisor/camera/color/image_raw'), Image, queue_size=2, buff_size=2**24)
        self.depth_sub = message_filters.Subscriber(camera.get('depth_topic', '/supervisor/camera/depth/image_raw'), Image, queue_size=2, buff_size=2**24)
        self.sync = message_filters.TimeSynchronizer([self.color_sub, self.depth_sub], 8)
        self.sync.registerCallback(self.frame)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(0.5, float(cfg.get('detect_hz', 5.0)))), self.process)
        self.status('inactive', 'waiting_for_unknown_selection')

    def selection(self, message):
        try:
            selection = json.loads(message.data)
            mode = selection['mode']
            generation, stamp = int(selection['generation']), int(selection['stamp_ns'])
            if mode not in ('carton', 'unknown') or generation < 0 or stamp <= 0:
                raise ValueError('invalid selection')
        except (KeyError, TypeError, ValueError) as exc:
            rospy.logwarn('Unknown perception ignored invalid selection: %s', exc)
            return
        with self.lock:
            if generation < self.generation or (generation == self.generation and mode != self.mode):
                return
            if (generation, mode) == (self.generation, self.mode):
                return
            with self.input_lock:
                self.mode, self.generation, self.selection_ns = mode, generation, stamp
                self.latest, self.last_consumed_ns = None, 0
                self.last_receive_wall = time.monotonic()
            self.anchor_source_stamp_ns = 0
            self.tracker.reset()
            self.acquisition.reset()
            self.unavailable('waiting_for_exact_rgbd' if mode == 'unknown' else 'carton_selected',
                             rospy.Time(stamp // 1000000000, stamp % 1000000000), lose=False)

    def frame(self, color, depth):
        with self.input_lock:
            stamp_ns = color.header.stamp.to_nsec()
            if self.mode != 'unknown' or stamp_ns <= self.selection_ns:
                return
            # TimeSynchronizer is exact; retain a defensive integer check.
            if stamp_ns != depth.header.stamp.to_nsec() or stamp_ns <= self.last_consumed_ns:
                return
            if self.latest is not None and stamp_ns <= self.latest[0].header.stamp.to_nsec():
                return
            self.latest = (color, depth)
            self.last_receive_wall = time.monotonic()

    def status(self, state, detail, source=None, candidate=None, elapsed=None):
        payload = dict(mode='unknown', generation=self.generation,
                       selection_stamp_ns=self.selection_ns, state=state, detail=detail,
                       label='unknown_small', confidence_kind='geometry_quality',
                       semantic_probability=None, target_locked=self.tracker.anchor is not None,
                       target_lost=self.tracker.lost)
        if self.tracker.anchor is not None:
            payload.update(anchor_source_stamp_ns=self.anchor_source_stamp_ns,
                           anchor_position_base=self.tracker.anchor.position_base.tolist(),
                           anchor_extent_m=self.tracker.anchor.extent_m.tolist(),
                           reference_support_plane_base=self.tracker.plane_base.tolist(),
                           reference_support_footprint_base=self.tracker.anchor.support_footprint_base.tolist())
        if source is not None:
            color, depth = source
            payload.update(source_stamp_ns=color.header.stamp.to_nsec(),
                           color_stamp_ns=color.header.stamp.to_nsec(),
                           depth_stamp_ns=depth.header.stamp.to_nsec(),
                           color_frame_id=color.header.frame_id,
                           depth_frame_id=depth.header.frame_id,
                           color_seq=int(color.header.seq), depth_seq=int(depth.header.seq),
                           exact_rgbd=True, tf_lookup_stamp_ns=color.header.stamp.to_nsec(),
                           projection_frame_convention='ros_camera_link')
        if candidate is not None:
            payload.update(geometry_quality=candidate.quality,
                           geometry_metrics=candidate.metrics,
                           position_base=candidate.position_base.tolist())
        if elapsed is not None:
            payload['processing_seconds'] = float(elapsed)
        self.pubs['detector_status'].publish(String(data=json.dumps(payload, sort_keys=True)))

    def unavailable(self, reason, stamp=None, source=None, lose=True):
        if lose:
            self.tracker.unavailable()
        stamp = stamp or rospy.Time.now()
        obj = ObjectPose()
        obj.header.stamp, obj.header.frame_id = stamp, self.base_frame
        obj.label, obj.detected = 'unknown_small', False
        mask = self.bridge.cv2_to_imgmsg(np.zeros(self.last_shape, dtype=np.uint8), encoding='mono8')
        mask.header.stamp, mask.header.frame_id = stamp, self.camera_frame
        self.pubs['object_mask'].publish(mask)
        self.pubs['object'].publish(obj)
        self.pubs['object_detected'].publish(Bool(data=False))
        self.pubs['raw_object_detected'].publish(Bool(data=False))
        self.status('unavailable' if self.mode == 'unknown' else 'inactive', reason, source)

    def process(self, _event):
        with self.lock:
            with self.input_lock:
                if self.mode != 'unknown':
                    return
                source, self.latest = self.latest, None
                generation = self.generation
                receive_wall = self.last_receive_wall
                if source is not None:
                    self.last_consumed_ns = source[0].header.stamp.to_nsec()
            if source is None:
                if time.monotonic() - receive_wall > self.max_age:
                    self.unavailable('exact_rgbd_stream_stale')
                return
            color, depth_message = source
            anchor = self.tracker.anchor
            last = self.tracker.last
            plane = None if self.tracker.plane_base is None else self.tracker.plane_base.copy()
        started = time.monotonic()
        try:
            age = (rospy.Time.now() - color.header.stamp).to_sec()
            if not (0 <= age <= self.max_age):
                raise ValueError('source_frame_stale_or_future')
            if (color.header.frame_id != self.camera_frame
                    or depth_message.header.frame_id != self.camera_frame):
                raise ValueError('aligned_rgbd_frame_mismatch')
            if (color.width, color.height) != (depth_message.width, depth_message.height):
                raise ValueError('unaligned_rgbd_dimensions')
            camera = rospy.get_param('/camera', {})
            if not bool(camera.get('align_depth_to_color', False)):
                raise ValueError('aligned_depth_required')
            intrinsics = [float(camera[name]) for name in ('fx', 'fy', 'cx', 'cy')]
            array = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding='passthrough')
            if depth_message.encoding in ('16UC1', 'mono16'):
                scale = float(camera['depth_scale'])
                if not np.isfinite(scale) or scale <= 0:
                    raise ValueError('invalid_runtime_depth_scale')
                depth = np.asarray(array, dtype=np.float32) * scale
            elif depth_message.encoding == '32FC1':
                depth = np.asarray(array, dtype=np.float32)
            else:
                raise ValueError('unsupported_depth_encoding')
            transform = self.buffer.lookup_transform(self.base_frame, self.camera_frame,
                                                     color.header.stamp, rospy.Duration(self.tf_wait))
            result = segment_with_support_fallback(
                depth, intrinsics, transform_matrix(transform), self.config,
                anchor=anchor, last=last, reference_plane_base=plane)
            with self.lock:
                if generation != self.generation or self.mode != 'unknown':
                    return
                self.last_shape = depth.shape
                initial_refined = None
                if self.tracker.anchor is None:
                    # A one-frame depth fragment must not become an immutable
                    # identity. Confirm it before the real tracker is mutated.
                    provisional = TargetTracker(self.config)
                    initial, reason = provisional.choose(result, depth.shape)
                    if initial is None or initial.quality < self.minimum_quality:
                        self.acquisition.reset()
                        self.unavailable(reason if initial is None else 'insufficient_geometry_quality',
                                         color.header.stamp, source, lose=False)
                        return
                    try:
                        initial_refined = (refine_candidate(
                            self.bridge.imgmsg_to_cv2(color, desired_encoding='bgr8'),
                            depth, intrinsics, transform_matrix(transform), initial, self.config)
                            if self.rgb_instance_refinement else initial)
                    except ValueError as exc:
                        self.acquisition.reset()
                        self.unavailable('initial_instance_' + str(exc), color.header.stamp, source, lose=False)
                        return
                    confirmed, detail = self.acquisition.observe(initial, initial_refined, color.header.stamp.to_nsec())
                    if not confirmed:
                        self.unavailable(detail, color.header.stamp, source, lose=False)
                        return
                candidate, reason = self.tracker.choose(result, depth.shape)
                if candidate is not None and self.anchor_source_stamp_ns == 0:
                    self.anchor_source_stamp_ns = color.header.stamp.to_nsec()
                if candidate is None:
                    self.unavailable(reason, color.header.stamp, source)
                    return
                if candidate.quality < self.minimum_quality:
                    self.unavailable('insufficient_geometry_quality', color.header.stamp, source)
                    return
                if initial_refined is not None:
                    candidate = initial_refined
                elif self.rgb_instance_refinement:
                    try:
                        candidate = refine_candidate(
                            self.bridge.imgmsg_to_cv2(color, desired_encoding='bgr8'),
                            depth, intrinsics, transform_matrix(transform), candidate, self.config)
                    except ValueError as exc:
                        # A failed RGB-D ownership estimate must not publish
                        # the contaminated depth proposal as an accepted mask.
                        # Keep identity while withholding this frame; fresh
                        # measurements must still pass the original fusion gates.
                        self.unavailable('rgb_depth_refinement_rejected: ' + str(exc),
                                         color.header.stamp, source, lose=False)
                        return
                # Frame identity remains that of the RGB-D source, including mask.
                obj = ObjectPose()
                obj.header.stamp, obj.header.frame_id = color.header.stamp, self.base_frame
                obj.header.seq = color.header.seq
                obj.detected, obj.label, obj.confidence = True, 'unknown_small', candidate.quality
                obj.u, obj.v = [int(round(value)) for value in candidate.center_uv]
                obj.bbox_x, obj.bbox_y, obj.bbox_width, obj.bbox_height = candidate.bbox
                obj.depth_m = candidate.depth_m
                for name, point, frame_id in (
                        ('pose_camera', candidate.position_camera, self.camera_frame),
                        ('pose_base', candidate.position_base, self.base_frame)):
                    pose = PoseStamped()
                    pose.header.stamp, pose.header.frame_id = color.header.stamp, frame_id
                    pose.header.seq = color.header.seq
                    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = point
                    pose.pose.orientation.w = 1.0
                    setattr(obj, name, pose)
                    self.pubs['object_'+name].publish(pose)
                mask = self.bridge.cv2_to_imgmsg(candidate.mask, encoding='mono8')
                mask.header = color.header
                self.pubs['object_mask'].publish(mask)
                self.pubs['object'].publish(obj)
                self.pubs['object_detected'].publish(Bool(data=True))
                self.pubs['raw_object_detected'].publish(Bool(data=True))
                self.status('ready', reason, source, candidate, time.monotonic()-started)
        except Exception as exc:
            with self.lock:
                if generation == self.generation and self.mode == 'unknown':
                    if self.tracker.anchor is None:
                        self.acquisition.reset()
                    reason = ('missing_exact_tf: ' if isinstance(exc, (
                        tf2_ros.LookupException, tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException)) else 'perception_failure: ') + str(exc)
                    self.unavailable(reason, color.header.stamp, source)
                    rospy.logwarn_throttle(3.0, 'Unknown perception unavailable: %s', reason)


if __name__ == '__main__':
    rospy.init_node('unknown_tabletop_perception')
    UnknownTabletopPerception()
    rospy.spin()
