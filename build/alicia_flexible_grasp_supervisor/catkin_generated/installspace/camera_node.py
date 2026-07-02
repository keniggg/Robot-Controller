#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
try:
    from cv_bridge import CvBridge
except Exception:
    CvBridge = None
from alicia_flexible_grasp.vision.realsense_manager import RealSenseManager

class CameraNode:
    def __init__(self):
        self.cfg = rospy.get_param('/camera', {})
        self.color_topic = self.cfg.get('color_topic', '/supervisor/camera/color/image_raw')
        self.depth_topic = self.cfg.get('depth_topic', '/supervisor/camera/depth/image_raw')
        self.frame_id = self.cfg.get('frame_id', 'camera_color_optical_frame')
        self.width = self.cfg.get('width', 640)
        self.height = self.cfg.get('height', 480)
        self.fps = self.cfg.get('fps', 30)
        self.align_depth_to_color = self.cfg.get('align_depth_to_color', True)
        self.fallback_to_simulation = bool(self.cfg.get('fallback_to_simulation', False))
        self.read_failure_limit = max(1, int(self.cfg.get('read_failure_limit', 3)))
        self._read_failures = 0
        self.bridge = CvBridge() if CvBridge else None
        self.pub_color = rospy.Publisher(self.color_topic, Image, queue_size=2)
        self.pub_depth = rospy.Publisher(self.depth_topic, Image, queue_size=2)
        simulate = bool(self.cfg.get('simulate', False))
        self.cam = None
        self._camera_started = False
        self._start_camera_or_defer(simulate)
        self.rate = rospy.Rate(float(self.fps))

    def _make_camera(self, simulate):
        return RealSenseManager(
            self.width,
            self.height,
            self.fps,
            self.align_depth_to_color,
            simulate=simulate
        )

    def _start_camera(self, simulate):
        self.cam = self._make_camera(simulate)
        self.cam.start()
        self._camera_started = True
        self._publish_runtime_camera_params()
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

    def publish_image(self, pub, cv_img, encoding):
        if self.bridge is None:
            return
        msg = self.bridge.cv2_to_imgmsg(cv_img, encoding=encoding)
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        pub.publish(msg)

    def _publish_runtime_camera_params(self):
        if hasattr(self.cam, 'depth_scale'):
            depth_scale = float(self.cam.depth_scale)
            rospy.set_param('/camera/depth_scale', depth_scale)
            rospy.loginfo('Camera depth scale set to %.7f m/unit', depth_scale)

    def spin(self):
        while not rospy.is_shutdown():
            if not self._camera_started:
                if not self._start_camera_or_defer(False):
                    self.rate.sleep()
                    continue
            try:
                color, depth = self.cam.read()
                self._read_failures = 0
            except Exception as exc:
                recovered = self._recover_from_read_error(exc)
                if not recovered:
                    self.rate.sleep()
                continue
            if color is not None:
                self.publish_image(self.pub_color, color, 'bgr8')
            if depth is not None:
                self.publish_image(self.pub_depth, depth, '16UC1')
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('camera_node')
    node = CameraNode()
    rospy.on_shutdown(node.shutdown)
    node.spin()
