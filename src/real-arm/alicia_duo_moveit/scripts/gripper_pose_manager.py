#!/usr/bin/env python3
# filepath: /home/xuanya/alicia_ws/src/alicia_duo_moveit/scripts/gripper_pose_manager.py

import rospy
import numpy as np
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from tf.transformations import (
    quaternion_matrix,
    quaternion_from_matrix,
    inverse_matrix,
    compose_matrix,
    decompose_matrix
)

class GripperPoseManager:
    def __init__(self):
        """Initialize the gripper pose manager with TF capabilities"""
        # Set up TF listener
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.0))  # Buffer for 5 seconds
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # This is the manipulator end-effector frame (where MoveIt commands are sent)
        self.manipulator_frame = "Link06"
        
        # This is the TCP frame (where target poses are typically specified)
        self.tcp_frame = "tcp_link"
        
        # Wait for the transform to become available
        self.wait_for_transform() 
        
        rospy.loginfo("GripperPoseManager initialized")
    
    def wait_for_transform(self, timeout=5.0):
        """Wait for the transform between manipulator frame and TCP frame"""
        start_time = rospy.Time.now()
        rate = rospy.Rate(10.0)  # 10 Hz
        
        rospy.loginfo(f"Waiting for transform from {self.manipulator_frame} to {self.tcp_frame}...")
        
        while not rospy.is_shutdown():
            try:
                # Check if transform is available
                self.tf_buffer.lookup_transform(
                    self.manipulator_frame,
                    self.tcp_frame,
                    rospy.Time(0)
                )
                rospy.loginfo(f"Transform from {self.manipulator_frame} to {self.tcp_frame} is available")
                return True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                if (rospy.Time.now() - start_time).to_sec() > timeout:
                    rospy.logwarn(f"Timeout waiting for transform from {self.manipulator_frame} to {self.tcp_frame}")
                    return False
                rate.sleep()
    
    def get_transform_matrix(self, target_frame, source_frame, time=None):
        """
        Get the transform matrix from source_frame to target_frame
        
        Args:
            target_frame: The frame to transform to
            source_frame: The frame to transform from
            time: The time at which to get the transform (defaults to latest)
            
        Returns:
            numpy.ndarray: 4x4 homogeneous transformation matrix, or None on failure
        """
        if time is None:
            time = rospy.Time(0)  # Latest transform
            
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                time
            )
            
            # Convert to 4x4 matrix
            trans = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ]
            
            quat = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]
            
            # Create homogeneous transformation matrix
            matrix = quaternion_matrix(quat)
            matrix[0:3, 3] = trans
            
            return matrix
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Failed to get transform from {source_frame} to {target_frame}: {e}")
            return None
    
    def tcp_to_manipulator_pose(self, tcp_pose, base_frame="world"):
        """
        Convert a TCP pose to the corresponding manipulator pose
        
        Args:
            tcp_pose: geometry_msgs.msg.Pose or PoseStamped - Target pose for the TCP
            base_frame: The frame the poses are expressed in (default: "world")
            
        Returns:
            geometry_msgs.msg.Pose - The corresponding pose for the manipulator link
        """
        # Extract pose and frame if it's a PoseStamped
        if isinstance(tcp_pose, PoseStamped):
            pose = tcp_pose.pose
            pose_frame = tcp_pose.header.frame_id
        else:
            pose = tcp_pose
            pose_frame = base_frame
            
        # Get the transform from manipulator to TCP
        T_manipulator_tcp = self.get_transform_matrix(self.manipulator_frame, self.tcp_frame)
        
        if T_manipulator_tcp is None:
            rospy.logerr("Failed to get transform, cannot convert TCP pose to manipulator pose")
            return None
            
        # Create a transformation matrix from the TCP target pose
        tcp_target_matrix = compose_matrix(
            translate=[pose.position.x, pose.position.y, pose.position.z],
            angles=[0, 0, 0],  # We'll set rotation separately from quaternion
        )
        
        # Set rotation from quaternion
        quat_matrix = quaternion_matrix([
            pose.orientation.x, 
            pose.orientation.y, 
            pose.orientation.z, 
            pose.orientation.w
        ])
        tcp_target_matrix[:3, :3] = quat_matrix[:3, :3]
        
        # Calculate the manipulator pose that would place the TCP at the target pose
        # manipulator_pose = tcp_pose * inv(tcp_T_manipulator)
        T_tcp_manipulator = inverse_matrix(T_manipulator_tcp)
        manipulator_matrix = np.dot(tcp_target_matrix, T_tcp_manipulator)
        
        # Convert the matrix back to a pose
        manipulator_pose = Pose()
        manipulator_pose.position.x = manipulator_matrix[0, 3]
        manipulator_pose.position.y = manipulator_matrix[1, 3]
        manipulator_pose.position.z = manipulator_matrix[2, 3]
        
        # Get the quaternion from the rotation matrix
        quat = quaternion_from_matrix(manipulator_matrix)
        manipulator_pose.orientation.x = quat[0]
        manipulator_pose.orientation.y = quat[1]
        manipulator_pose.orientation.z = quat[2]
        manipulator_pose.orientation.w = quat[3]
        
        # Return PoseStamped if input was PoseStamped
        if isinstance(tcp_pose, PoseStamped):
            result = PoseStamped()
            result.header.frame_id = pose_frame
            result.header.stamp = rospy.Time.now()
            result.pose = manipulator_pose
            return result
        
        return manipulator_pose
        
    def manipulator_to_tcp_pose(self, manipulator_pose, base_frame="world"):
        """
        Convert a manipulator pose to the corresponding TCP pose
        
        Args:
            manipulator_pose: geometry_msgs.msg.Pose or PoseStamped - Pose for the manipulator
            base_frame: The frame the poses are expressed in (default: "world")
            
        Returns:
            geometry_msgs.msg.Pose - The corresponding pose for the TCP
        """
        # Extract pose and frame if it's a PoseStamped
        if isinstance(manipulator_pose, PoseStamped):
            pose = manipulator_pose.pose
            pose_frame = manipulator_pose.header.frame_id
        else:
            pose = manipulator_pose
            pose_frame = base_frame
            
        # Get the transform from manipulator to TCP
        T_manipulator_tcp = self.get_transform_matrix(self.manipulator_frame, self.tcp_frame)
        
        if T_manipulator_tcp is None:
            rospy.logerr("Failed to get transform, cannot convert manipulator pose to TCP pose")
            return None
            
        # Create a transformation matrix from the manipulator pose
        manipulator_matrix = compose_matrix(
            translate=[pose.position.x, pose.position.y, pose.position.z],
            angles=[0, 0, 0],  # We'll set rotation separately from quaternion
        )
        
        # Set rotation from quaternion
        quat_matrix = quaternion_matrix([
            pose.orientation.x, 
            pose.orientation.y, 
            pose.orientation.z, 
            pose.orientation.w
        ])
        manipulator_matrix[:3, :3] = quat_matrix[:3, :3]
        
        # Calculate the TCP pose that would result from the manipulator at the target pose
        # tcp_pose = manipulator_pose * tcp_T_manipulator
        tcp_matrix = np.dot(manipulator_matrix, T_manipulator_tcp)
        
        # Convert the matrix back to a pose
        tcp_pose = Pose()
        tcp_pose.position.x = tcp_matrix[0, 3]
        tcp_pose.position.y = tcp_matrix[1, 3]
        tcp_pose.position.z = tcp_matrix[2, 3]
        
        # Get the quaternion from the rotation matrix
        quat = quaternion_from_matrix(tcp_matrix)
        tcp_pose.orientation.x = quat[0]
        tcp_pose.orientation.y = quat[1]
        tcp_pose.orientation.z = quat[2]
        tcp_pose.orientation.w = quat[3]
        
        # Return PoseStamped if input was PoseStamped
        if isinstance(manipulator_pose, PoseStamped):
            result = PoseStamped()
            result.header.frame_id = pose_frame
            result.header.stamp = rospy.Time.now()
            result.pose = tcp_pose
            return result
        
        return tcp_pose