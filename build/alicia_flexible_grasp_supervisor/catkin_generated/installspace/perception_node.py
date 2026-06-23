#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from alicia_flexible_grasp_supervisor.msg import ObjectPose
from alicia_flexible_grasp.vision.object_detector import HSVObjectDetector
from alicia_flexible_grasp.vision.depth_projector import pixel_to_camera_point
from alicia_flexible_grasp.utils.transform_utils import make_transform, transform_point, pose_from_xyz_quat


class PerceptionNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.color = None
        self.depth = None
        self.info = None
        def gp(ns, name, default):
            return rospy.get_param('~' + name, rospy.get_param('/' + ns + '/' + name, default))
        lower = gp('perception', 'hsv_lower', [35, 50, 50])
        upper = gp('perception', 'hsv_upper', [85, 255, 255])
        self.detector = HSVObjectDetector(lower, upper, gp('perception', 'min_area', 300))
        self.base_frame = gp('perception', 'base_frame', 'base_link')
        self.camera_frame = gp('perception', 'camera_frame', 'camera_color_optical_frame')
        trans = rospy.get_param('~handeye_translation', rospy.get_param('/handeye/translation', [0, 0, 0]))
        quat = rospy.get_param('~handeye_rotation_quat_xyzw', rospy.get_param('/handeye/rotation_quat_xyzw', [0, 0, 0, 1]))
        self.T_base_camera = make_transform(trans, quat)
        self.default_quat = rospy.get_param('~default_orientation_xyzw', [0, 1, 0, 0])
        self.pub = rospy.Publisher('/perception/object_pose_base', ObjectPose, queue_size=10)
        self.pub_detected = rospy.Publisher('/perception/object_detected', Bool, queue_size=10)
        rospy.Subscriber('/camera/color/image', Image, self.color_cb, queue_size=1)
        rospy.Subscriber('/camera/depth/image', Image, self.depth_cb, queue_size=1)
        rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.info_cb, queue_size=1)

    def color_cb(self, msg):
        self.color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def depth_cb(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

    def info_cb(self, msg):
        self.info = msg

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            out = ObjectPose()
            out.header.stamp = rospy.Time.now()
            out.header.frame_id = self.base_frame
            out.label = 'target'
            if self.color is None or self.depth is None or self.info is None:
                out.detected = False; out.status = 'waiting for color/depth/camera_info'
                self.pub.publish(out); self.pub_detected.publish(Bool(data=False)); rate.sleep(); continue
            det, _ = self.detector.detect(self.color)
            if not det:
                out.detected = False; out.status = 'no object'
                self.pub.publish(out); self.pub_detected.publish(Bool(data=False)); rate.sleep(); continue
            u, v = det['u'], det['v']
            raw = float(self.depth[v, u])
            # 16UC1 depth from RealSense is commonly millimeters.
            depth_m = raw * 0.001 if raw > 10 else raw
            K = self.info.K
            fx, fy, cx, cy = K[0], K[4], K[2], K[5]
            p_cam = pixel_to_camera_point(u, v, depth_m, fx, fy, cx, cy)
            p_base = transform_point(self.T_base_camera, [p_cam.x, p_cam.y, p_cam.z])
            out.detected = True
            out.confidence = float(det['confidence'])
            out.u = int(u); out.v = int(v); out.depth = float(depth_m)
            out.position_camera = p_cam
            out.pose_base = pose_from_xyz_quat(p_base, self.default_quat)
            out.status = 'ok'
            self.pub.publish(out); self.pub_detected.publish(Bool(data=True))
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('perception_node')
    PerceptionNode().spin()
