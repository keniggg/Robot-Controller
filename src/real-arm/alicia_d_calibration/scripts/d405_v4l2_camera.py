#!/usr/bin/env python3
import yaml

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image


def load_camera_info(path, frame_id):
    with open(path, "r") as f:
        data = yaml.safe_load(f)

    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.width = int(data["image_width"])
    msg.height = int(data["image_height"])
    msg.distortion_model = data.get("distortion_model", "plumb_bob")
    msg.D = list(data["distortion_coefficients"]["data"])
    msg.K = list(data["camera_matrix"]["data"])
    msg.R = list(data["rectification_matrix"]["data"])
    msg.P = list(data["projection_matrix"]["data"])
    return msg


def main():
    rospy.init_node("d405_v4l2_camera")

    device = rospy.get_param("~device", "/dev/video4")
    width = int(rospy.get_param("~width", 640))
    height = int(rospy.get_param("~height", 480))
    fps = float(rospy.get_param("~fps", 30.0))
    frame_id = rospy.get_param("~frame_id", "camera_link")
    camera_info_url = rospy.get_param("~camera_info_url", "")

    image_pub = rospy.Publisher("/camera/color/image_raw", Image, queue_size=1)
    info_pub = rospy.Publisher("/camera/color/camera_info", CameraInfo, queue_size=1)
    bridge = CvBridge()

    camera_info = load_camera_info(camera_info_url, frame_id)

    def open_camera():
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    cap = open_camera()
    if not cap.isOpened():
        rospy.logerr("Failed to open V4L2 camera device: %s", device)
        return

    rospy.loginfo("Publishing D405 V4L2 color stream from %s at %dx%d", device, width, height)
    rate = rospy.Rate(fps)
    failed_reads = 0

    while not rospy.is_shutdown():
        ok, frame = cap.read()
        if not ok or frame is None:
            failed_reads += 1
            rospy.logwarn_throttle(1.0, "Failed to read frame from %s", device)
            if failed_reads >= 10:
                rospy.logwarn("Reopening V4L2 camera device after repeated read failures: %s", device)
                cap.release()
                rospy.sleep(0.5)
                cap = open_camera()
                failed_reads = 0
            rate.sleep()
            continue
        failed_reads = 0

        stamp = rospy.Time.now()
        image_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = frame_id

        camera_info.header.stamp = stamp
        camera_info.header.frame_id = frame_id

        image_pub.publish(image_msg)
        info_pub.publish(camera_info)
        rate.sleep()

    cap.release()


if __name__ == "__main__":
    main()
