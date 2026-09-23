#!/usr/bin/env python3
import copy
import time

import numpy as np
import rospy
from sensor_msgs.msg import Image, JointState
try:
    from cv_bridge import CvBridge
except Exception:
    CvBridge = None
from alicia_flexible_grasp.vision.realsense_manager import RealSenseManager

class CameraNode:
    def __init__(self):
        self.cfg = copy.deepcopy(rospy.get_param('/camera', {}))
        self.color_topic = self.cfg.get('color_topic', '/supervisor/camera/color/image_raw')
        self.depth_topic = self.cfg.get('depth_topic', '/supervisor/camera/depth/image_raw')
        self.depth_preview_topic = str(self.cfg.get('depth_preview_topic', '') or '')
        self.depth_preview_width = max(1, int(self.cfg.get('depth_preview_width', 320)))
        self.depth_preview_height = max(1, int(self.cfg.get('depth_preview_height', 240)))
        self.frame_id = self.cfg.get('frame_id', 'camera_color_optical_frame')
        self.width = self.cfg.get('width', 640)
        self.height = self.cfg.get('height', 480)
        self.fps = self.cfg.get('fps', 30)
        self.publish_fps = min(
            float(self.fps),
            max(0.1, float(self.cfg.get('publish_fps', self.fps))),
        )
        self._publish_period_sec = 1.0 / self.publish_fps
        self._last_publish_monotonic = None
        self._monotonic = time.monotonic
        self.align_depth_to_color = self.cfg.get('align_depth_to_color', True)
        self.fallback_to_simulation = bool(self.cfg.get('fallback_to_simulation', False))
        self.read_failure_limit = max(1, int(self.cfg.get('read_failure_limit', 3)))
        self._read_failures = 0
        self.bridge = CvBridge() if CvBridge else None
        self.pub_color = rospy.Publisher(self.color_topic, Image, queue_size=2)
        self.pub_depth = rospy.Publisher(self.depth_topic, Image, queue_size=2)
        self.pub_depth_preview = (
            rospy.Publisher(self.depth_preview_topic, Image, queue_size=1)
            if self.depth_preview_topic else None
        )
        simulate = bool(self.cfg.get('simulate', False))
        self.cam = None
        self._stationary_window = None
        self._last_temporal_status = None
        if self.cfg.get('mode_depth_preset', {}).get('stationary_temporal_enabled', False):
            from alicia_flexible_grasp.vision.stationary_depth import StationaryDepthWindow
            self._stationary_window = StationaryDepthWindow()
            self._stationary_sdk_sub = rospy.Subscriber(
                '/alicia_d/sdk_command', JointState,
                lambda msg: self._stationary_window.observe('sdk', msg), queue_size=10)
            self._stationary_encoder_sub = rospy.Subscriber(
                '/alicia_d/accepted_joint_states', JointState,
                lambda msg: self._stationary_window.observe('accepted', msg), queue_size=10)
        self._camera_started = False
        self._start_camera_or_defer(simulate)
        self.rate = rospy.Rate(float(self.fps))

    def _make_camera(self, simulate):
        depth_filter_cfg = dict(self.cfg.get('depth_filter', {}) or {})
        perception_cfg = rospy.get_param('/perception', {})
        depth_filter_cfg.setdefault('depth_min_m', perception_cfg.get('depth_min_m', 0.03))
        depth_filter_cfg.setdefault('depth_max_m', perception_cfg.get('depth_max_m', 2.0))
        camera_args = dict(
            width=self.width,
            height=self.height,
            fps=self.fps,
            align_depth_to_color=self.align_depth_to_color,
            simulate=simulate,
            depth_filter_cfg=depth_filter_cfg,
        )
        projection_cfg = dict(self.cfg.get('color_projection_correction', {}) or {})
        if projection_cfg.get('enabled', False):
            target = projection_cfg['sdk_intrinsics']
            for key in ('fx', 'fy', 'cx', 'cy'):
                if not np.isclose(float(self.cfg[key]), float(target[key]), rtol=0, atol=1e-6):
                    raise ValueError('camera/%s must match the corrected RGB-D SDK projection' % key)
            camera_args['color_projection_cfg'] = projection_cfg
        preset_cfg = dict(self.cfg.get('mode_depth_preset', {}) or {})
        if preset_cfg.get('enabled', False):
            if simulate:
                raise ValueError('mode depth preset requires a real camera')
            camera_args['mode_depth_preset_cfg'] = preset_cfg
        return RealSenseManager(**camera_args)

    def _start_camera(self, simulate):
        self.cam = self._make_camera(simulate)
        self.cam.start()
        if getattr(self, '_stationary_window', None) is not None:
            self.cam.stationary_depth_provider = lambda: self._stationary_window.stationary(rospy.get_time())
        self._sync_depth_preset(publish=False)
        self._publish_runtime_camera_params()
        self._camera_started = True
        mode = 'simulated' if simulate else 'real'
        rospy.loginfo('Camera started (%s): color=%s depth=%s', mode, self.color_topic, self.depth_topic)
        return True

    def _start_camera_or_defer(self, simulate):
        try:
            return self._start_camera(simulate)
        except Exception as exc:
            self._camera_started = False
            self._stop_camera()
            if self.fallback_to_simulation and not simulate:
                rospy.logerr('Camera start failed: %s. Falling back to simulated camera.', exc)
                return self._start_camera(True)
            rospy.logerr(
                'Camera start failed: %s. Node stays alive and will retry real camera; '
                'check for stale camera_node/RealSense viewers if the device is busy.',
                exc,
            )
            return False

    def _stop_camera(self):
        if self.cam is None:
            return
        try:
            self.cam.stop()
        except Exception as exc:
            rospy.logwarn('Camera stop failed during recovery: %s', exc)
        finally:
            self._camera_started = False

    def shutdown(self):
        self._stop_camera()

    def _recover_from_read_error(self, exc):
        self._read_failures += 1
        rospy.logwarn_throttle(
            2.0,
            'Camera read failed (%d/%d): %s',
            self._read_failures,
            self.read_failure_limit,
            exc
        )
        if self._read_failures < self.read_failure_limit:
            return False
        self._stop_camera()
        if self.fallback_to_simulation:
            self._start_camera(True)
            self._read_failures = 0
            rospy.logwarn('Camera stream switched to simulated fallback after read failures.')
            return True
        try:
            self._start_camera(False)
            self._read_failures = 0
            rospy.logwarn('Camera stream restarted after read failures.')
            return True
        except Exception as restart_exc:
            rospy.logerr(
                'Camera restart failed after read error: %s. Keeping real camera mode; no simulated frames will be published.',
                restart_exc
            )
            self._read_failures = self.read_failure_limit
            return False

    def publish_image(self, pub, cv_img, encoding, stamp):
        if self.bridge is None:
            return
        msg = self.bridge.cv2_to_imgmsg(cv_img, encoding=encoding)
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        pub.publish(msg)

    @staticmethod
    def _resize_depth_preview(depth, width, height):
        array = np.asanyarray(depth)
        source_height, source_width = array.shape[:2]
        width = max(1, int(width))
        height = max(1, int(height))
        if (source_width, source_height) == (width, height):
            return array
        x_indices = np.linspace(
            0,
            source_width - 1,
            width,
        ).astype(np.intp)
        y_indices = np.linspace(
            0,
            source_height - 1,
            height,
        ).astype(np.intp)
        return array[np.ix_(y_indices, x_indices)]

    def publish_depth_preview(self, depth, stamp):
        pub = self.pub_depth_preview
        if pub is None:
            return
        connection_count = getattr(pub, 'get_num_connections', lambda: 1)()
        if int(connection_count) <= 0:
            return
        preview = self._resize_depth_preview(
            depth,
            self.depth_preview_width,
            self.depth_preview_height,
        )
        self.publish_image(pub, preview, '16UC1', stamp)

    def _publish_runtime_camera_params(self):
        # One namespace replacement publishes K and scale together before the
        # first image, including after a stream recovery. Keep unrelated config.
        current = copy.deepcopy(rospy.get_param('/camera', self.cfg))
        if hasattr(self.cam, 'depth_scale'):
            current['depth_scale'] = float(self.cam.depth_scale)
        if bool(getattr(self.cam, 'simulate', False)):
            current['runtime_profile'] = {'source': 'simulation', 'schema_version': 1}
            current['intrinsics_source'] = 'configured_simulation'
        else:
            profile = copy.deepcopy(self.cam.runtime_profile)
            if profile.get('source') != 'realsense_active_profile':
                raise ValueError('real camera has no active SDK geometry profile')
            intrinsic = profile['geometry_intrinsics']
            for key in ('fx', 'fy', 'cx', 'cy'):
                current[key] = float(intrinsic[key])
            if (int(intrinsic['width']) != int(self.width)
                    or int(intrinsic['height']) != int(self.height)):
                raise ValueError('active SDK geometry dimensions differ from requested images')
            current['runtime_profile'] = profile
            current['intrinsics_source'] = profile['geometry_intrinsics_source']
        rospy.set_param('/camera', current)
        self.cfg = copy.deepcopy(current)
        rospy.loginfo('Camera runtime geometry: source=%s fx=%.6f fy=%.6f depth_scale=%.7f m/unit',
                      current['intrinsics_source'], float(current.get('fx', 0.)),
                      float(current.get('fy', 0.)), float(current.get('depth_scale', 0.)))

    def _sync_depth_preset(self, publish=True):
        if not self.cfg.get('mode_depth_preset', {}).get('enabled', False):
            return
        selection = rospy.get_param('/grasp_mode/selection', {'mode': 'carton'})
        mode = selection.get('mode')
        if self.cam.set_depth_preset_mode(mode) and publish:
            self._publish_runtime_camera_params()

    def _publication_is_due(self):
        now = self._monotonic()
        previous = self._last_publish_monotonic
        if previous is None or now < previous:
            self._last_publish_monotonic = now
            return True

        # Keep the ideal publication clock instead of rebasing it to the
        # hardware-frame timestamp. Rebasing quantizes a 30 FPS camera with a
        # 20 FPS publication target to every second frame (about 15 FPS).
        # Advancing the scheduled clock produces the intended alternating
        # one-frame/two-frame cadence without publishing bursts after a stall.
        scheduled = previous + self._publish_period_sec
        epsilon = min(1e-6, self._publish_period_sec * 1e-3)
        if now + epsilon < scheduled:
            return False
        elapsed_periods = max(
            0,
            int((now - scheduled + epsilon) / self._publish_period_sec),
        )
        self._last_publish_monotonic = (
            scheduled + elapsed_periods * self._publish_period_sec
        )
        return True

    def _pace_simulated_camera(self):
        if bool(getattr(self.cam, 'simulate', False)):
            self.rate.sleep()

    def spin(self):
        while not rospy.is_shutdown():
            if not self._camera_started:
                if not self._start_camera_or_defer(False):
                    self.rate.sleep()
                    continue
            try:
                self._sync_depth_preset()
                color, depth = self.cam.read()
                temporal_status = getattr(self.cam, 'runtime_profile', {}).get('stationary_temporal')
                if temporal_status != getattr(self, '_last_temporal_status', None):
                    self._publish_runtime_camera_params()
                    self._last_temporal_status = copy.deepcopy(temporal_status)
                self._read_failures = 0
            except Exception as exc:
                recovered = self._recover_from_read_error(exc)
                if not recovered:
                    self.rate.sleep()
                continue
            if not self._publication_is_due():
                self._pace_simulated_camera()
                continue
            stamp = rospy.Time.now()
            if color is not None:
                self.publish_image(self.pub_color, color, 'bgr8', stamp)
            if depth is not None:
                self.publish_image(self.pub_depth, depth, '16UC1', stamp)
                self.publish_depth_preview(depth, stamp)
            # RealSense wait_for_frames() already blocks until the next hardware
            # frame. Sleeping again here halves the effective camera rate.
            # Simulated reads return immediately, so they still need pacing.
            self._pace_simulated_camera()

if __name__ == '__main__':
    rospy.init_node('camera_node')
    node = CameraNode()
    rospy.on_shutdown(node.shutdown)
    node.spin()
