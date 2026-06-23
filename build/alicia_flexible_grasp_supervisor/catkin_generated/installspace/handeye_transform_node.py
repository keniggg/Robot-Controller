#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


if __name__ == '__main__':
    rospy.init_node('handeye_transform_node')
    base = rospy.get_param('~base_frame', 'base_link')
    cam = rospy.get_param('~camera_frame', 'camera_color_optical_frame')
    t = rospy.get_param('~translation', [0, 0, 0])
    q = rospy.get_param('~rotation_quat_xyzw', [0, 0, 0, 1])
    br = tf2_ros.StaticTransformBroadcaster()
    msg = TransformStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = base
    msg.child_frame_id = cam
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = [float(x) for x in t]
    msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = [float(x) for x in q]
    br.sendTransform(msg)
    rospy.loginfo('Published static handeye transform %s -> %s', base, cam)
    rospy.spin()
