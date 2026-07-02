import math
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped
from tf.transformations import quaternion_matrix, quaternion_from_euler, quaternion_multiply


def matrix_from_xyz_quat(xyz, quat_xyzw):
    mat = quaternion_matrix(quat_xyzw)
    mat[0:3, 3] = np.array(xyz, dtype=float)
    return mat


def transform_point(point_xyz, translation_xyz, rotation_xyzw):
    mat = matrix_from_xyz_quat(translation_xyz, rotation_xyzw)
    p = np.array([point_xyz[0], point_xyz[1], point_xyz[2], 1.0])
    out = mat.dot(p)
    return out[:3].tolist()


def make_pose_stamped(frame_id, xyz, quat_xyzw, stamp=None):
    import rospy
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.header.stamp = stamp if stamp is not None else rospy.Time.now()
    ps.pose.position.x = float(xyz[0])
    ps.pose.position.y = float(xyz[1])
    ps.pose.position.z = float(xyz[2])
    ps.pose.orientation.x = float(quat_xyzw[0])
    ps.pose.orientation.y = float(quat_xyzw[1])
    ps.pose.orientation.z = float(quat_xyzw[2])
    ps.pose.orientation.w = float(quat_xyzw[3])
    return ps


def euler_delta_quat(droll, dpitch, dyaw):
    return quaternion_from_euler(droll, dpitch, dyaw)
