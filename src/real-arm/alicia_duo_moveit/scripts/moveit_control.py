#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
from tf.transformations import quaternion_from_euler
import tf2_ros
import geometry_msgs.msg
import numpy as np

class MoveItRobotController:
    def __init__(self, manipulator_group="alicia", gripper_group="gripper", velocity=0.6):
        # Initialize MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        if not rospy.get_node_uri():
            rospy.init_node('moveit_robot_controller', anonymous=True)  #anonymous关键字是  是否保留通信的最后一条信息至下一个通话链接

        # Robot and planning interface
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Move groups
        self.manipulator = moveit_commander.MoveGroupCommander(manipulator_group)
        # self.gripper = moveit_commander.MoveGroupCommander(gripper_group)

        # Set velocity scaling (0.0 to 1.0)
        self.manipulator.set_max_velocity_scaling_factor(velocity)
        # self.gripper.set_max_velocity_scaling_factor(velocity)
        # set the manximum acceleration scaling factor
        self.manipulator.set_max_acceleration_scaling_factor(0.5)
        self.manipulator.set_planner_id("RRTConnectkConfigDefault")  # 更快更稳定
        self.manipulator.set_planning_time(10.0)                     # 增加规划时间
        self.manipulator.set_num_planning_attempts(10)
        self.manipulator.set_goal_position_tolerance(0.01)
        self.manipulator.set_goal_orientation_tolerance(0.01)
        self.manipulator.allow_replanning(True)

        rospy.loginfo("MoveItRobotController initialized.")
        self.gripper_pub = rospy.Publisher('/gripper_control', Float32, queue_size=10)

    def gripper_control(self, value):
        # rate = rospy.Rate(10)  # 10 Hz
        for i in range(3):
            self.gripper_pub.publish(Float32(data=value))
            rospy.sleep(0.3)


    def move_to_pose(self, pose):
        self.manipulator.set_pose_target(pose)
        success, plan, _, _ = self.manipulator.plan(pose)
        if not success:
            rospy.logwarn("IK solution could not be found for pre-grasp pose.")
        else:
            rospy.loginfo("IK plan success.")

        success = self.manipulator.go(wait=True)
        self.manipulator.stop()
        self.manipulator.clear_pose_targets()
        return success


    # Add to the MoveItRobotController class
    def get_current_pose(self):
        """
        获取当前机械臂的位姿
        
        Returns:
            geometry_msgs.msg.Pose: 当前机械臂的位姿
        """
        return self.manipulator.get_current_pose().pose
    
    def move_to_tcp_pose(self, tcp_pose):
        """
        Move to a target pose specified for the TCP (tool center point)
        
        Args:
            tcp_pose: geometry_msgs.msg.Pose - The desired pose for the TCP
            
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            # Import GripperPoseManager within the function to avoid circular imports
            from gripper_pose_manager import GripperPoseManager
            
            # Create a pose manager instance
            pose_manager = GripperPoseManager()
            
            # Convert TCP pose to Link06 pose
            manipulator_pose = pose_manager.tcp_to_manipulator_pose(tcp_pose)
            
            if manipulator_pose is None:
                rospy.logerr("Failed to convert TCP pose to manipulator pose")
                return False
            
            # Visualize both poses for debugging
            self.publish_object_tf(tcp_pose, "tcp_target")
            self.publish_object_tf(manipulator_pose, "link06_target")
            
            rospy.loginfo("TCP target: position=(%.4f, %.4f, %.4f)", 
                        tcp_pose.position.x, tcp_pose.position.y, tcp_pose.position.z)
            rospy.loginfo("Link06 target: position=(%.4f, %.4f, %.4f)", 
                        manipulator_pose.position.x, manipulator_pose.position.y, manipulator_pose.position.z)
            
            # Move to the converted pose
            return self.move_to_pose(manipulator_pose)
        
        except Exception as e:
            rospy.logerr(f"Error in move_to_tcp_pose: {e}")
            import traceback
            traceback.print_exc()
            return False
        

    def publish_object_tf(self, obj_pos, frame_id="pre_pose"):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"  # or "base_link" if that's your base
        t.child_frame_id = frame_id
        t.transform.translation.x = obj_pos.position.x
        t.transform.translation.y = obj_pos.position.y
        t.transform.translation.z = obj_pos.position.z

        t.transform.rotation.x = obj_pos.orientation.x 
        t.transform.rotation.y = obj_pos.orientation.y
        t.transform.rotation.z = obj_pos.orientation.z
        t.transform.rotation.w = obj_pos.orientation.w

        self.tf_broadcaster.sendTransform(t)

    
    def move_to_joint_state(self, joint_goals):
        self.manipulator.set_joint_value_target(joint_goals)
        success = self.manipulator.go(wait=True)
        self.manipulator.stop()
        return success

    def get_current_joint_state(self):
        """
        获取当前机械臂的关节状态
        
        Returns:
            list: 当前机械臂的关节角度列表
        """
        return self.manipulator.get_current_joint_values()
    def execute_trajectory(self, trajectory):
        """
        执行给定的机器人轨迹
        
        Args:
            trajectory (moveit_msgs.msg.RobotTrajectory): 要执行的轨迹对象
            
        Returns:
            bool: 执行成功返回True，否则返回False
        """
        try:
            rospy.loginfo("开始执行轨迹，共 %d 个点，时长 %.2f 秒", 
                        len(trajectory.joint_trajectory.points), 
                        trajectory.joint_trajectory.points[-1].time_from_start.to_sec())
            
            # 检查轨迹是否为空
            if not trajectory.joint_trajectory.points:
                rospy.logerr("轨迹为空，无法执行")
                return False
                
            # 验证关节名称是否与机器人模型匹配
            robot_joints = set(self.manipulator.get_active_joints())
            trajectory_joints = set(trajectory.joint_trajectory.joint_names)
            
            # 如果轨迹中包含的关节不在机器人关节列表中，发出警告
            if not trajectory_joints.issubset(robot_joints):
                rospy.logwarn("轨迹包含未知关节: %s", 
                            trajectory_joints.difference(robot_joints))
                rospy.logwarn("机器人有效关节: %s", robot_joints)
                
                # 尝试修复轨迹中的关节名称
                if len(trajectory_joints) == len(robot_joints):
                    rospy.logwarn("尝试修复轨迹关节名称...")
                    # 创建新的轨迹对象
                    fixed_trajectory = self._fix_joint_names(trajectory, list(robot_joints))
                    trajectory = fixed_trajectory
            
            # 执行轨迹
            result = self.manipulator.execute(trajectory, wait=True)
            
            # 停止移动并清理目标
            self.manipulator.stop()
            
            if result:
                rospy.loginfo("轨迹执行成功完成")
            else:
                rospy.logwarn("轨迹执行失败或被取消")
                
            return result
            
        except Exception as e:
            import traceback
            rospy.logerr("轨迹执行过程中发生错误: %s", e)
            traceback.print_exc()
            return False
            
    def _fix_joint_names(self, trajectory, correct_joint_names):
        """
        修复轨迹中的关节名称，使其与机器人模型匹配
        
        Args:
            trajectory: 原始轨迹对象
            correct_joint_names: 正确的关节名称列表
            
        Returns:
            修复后的轨迹对象
        """
        import copy
        
        # 创建新的轨迹对象
        fixed_trajectory = copy.deepcopy(trajectory)
        
        # 替换关节名称
        if len(fixed_trajectory.joint_trajectory.joint_names) == len(correct_joint_names):
            fixed_trajectory.joint_trajectory.joint_names = correct_joint_names
            rospy.loginfo("轨迹关节名称已修复")
        
        return fixed_trajectory

    def set_velocity_scaling_factor(self, factor):
        """
        设置机器人执行速度的缩放因子
        
        Args:
            factor (float): 速度缩放因子 (0.0 到 1.0)
            
        Returns:
            None
        """
        # 确保因子在有效范围内
        factor = max(0.01, min(1.0, factor))
        self.manipulator.set_max_velocity_scaling_factor(factor)
        rospy.loginfo("速度缩放因子设置为: %.2f", factor)
        

        
if __name__ == '__main__':
    controller = MoveItRobotController()

    # 定义初始位姿（当前）
    start_pose = controller.manipulator.get_current_pose().pose

    # 定义目标位姿变化：向上移动 0.1m，向右移动 0.05m
    # waypoints = []
    # waypoints.append(start_pose)

    # target_pose = Pose()
    # target_pose.position.x = start_pose.position.x + 0.05
    # target_pose.position.y = start_pose.position.y
    # target_pose.position.z = start_pose.position.z + 0.01
    # target_pose.orientation = start_pose.orientation  # 保持姿态不变

    # waypoints.append(target_pose)

    start_pose = controller.manipulator.get_current_pose().pose

    # Define target pose change: move upwards by 0.1m, right by 0.05m
    waypoints = []
    waypoints.append(start_pose)

    target_pose = Pose()
    target_pose.position.x = start_pose.position.x + 0.05
    target_pose.position.y = start_pose.position.y
    target_pose.position.z = start_pose.position.z + 0.01
    target_pose.orientation = start_pose.orientation  # Keep the same orientation

    waypoints.append(target_pose)

    # Plan Cartesian path
    (plan, fraction) = controller.manipulator.compute_cartesian_path(
        waypoints,            # Path waypoints
        eef_step=0.01,        # End effector step size in meters
        avoid_collisions=True # Avoid collisions
    )

    # If planning is successful, execute the trajectory
    if fraction >= 0.8:
        rospy.loginfo("Cartesian path planning completed with success: %.2f%%", fraction * 100.0)

        # Make sure the waypoints have increasing timestamps before executing the trajectory
        for idx, waypoint in enumerate(plan.joint_trajectory.points):
            waypoint.time_from_start = rospy.Duration(0.0) if idx == 0 else plan.joint_trajectory.points[idx - 1].time_from_start + rospy.Duration(0.1)

        # Execute trajectory
        success = controller.execute_trajectory(plan)
        if success:
            rospy.loginfo("Cartesian trajectory execution succeeded")
        else:
            rospy.logwarn("Cartesian trajectory execution failed")
    else:
        rospy.logwarn("Cartesian path planning success rate below 80%, aborting execution")

