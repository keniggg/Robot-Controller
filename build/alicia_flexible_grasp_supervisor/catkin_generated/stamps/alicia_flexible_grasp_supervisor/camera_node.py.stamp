#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from alicia_flexible_grasp.vision.realsense_manager import RealSenseManager


class CameraNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.width = rospy.get_param('~color_width', 640)
        self.height = rospy.get_param('~color_height', 480)
        self.fps = rospy.get_param('~fps', 30)
        self.align = rospy.get_param('~align_depth_to_color', True)
        self.pub_color = rospy.Publisher('/camera/color/image', Image, queue_size=2)
        self.pub_depth = rospy.Publisher('/camera/depth/image', Image, queue_size=2)
        self.cam = RealSenseManager(self.width, self.height, self.fps, self.align)

    def spin(self):
        self.cam.start()
        rate = rospy.Rate(self.fps)
        try:
            while not rospy.is_shutdown():
                color, depth, _ = self.cam.frames()
                if color is not None:
                    self.pub_color.publish(self.bridge.cv2_to_imgmsg(color, encoding='bgr8'))
                if depth is not None:
                    self.pub_depth.publish(self.bridge.cv2_to_imgmsg(depth, encoding='16UC1'))
                rate.sleep()
        finally:
            self.cam.stop()


if __name__ == '__main__':
    rospy.init_node('camera_node')
    CameraNode().spin()
