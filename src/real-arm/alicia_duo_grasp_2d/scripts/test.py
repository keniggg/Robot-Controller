#!/usr/bin/env python3
import rospy
import numpy as np
import time
from geometry_msgs.msg import Pose
from obpose import ObjectPoseTransformer
import geometry_msgs.msg
import tf_conversions
import tf2_ros

# from alicia_duo_grasp_2d.scripts.obpose import ObjectPoseTransformer
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float32
import sys
import os
robot_path = os.path.expanduser('~/alicia_ws/src/alicia_duo_moveit/scripts')
sys.path.append(robot_path)
from moveit_control import MoveItRobotController


controller = MoveItRobotController()
currentpose = controller.get_current_pose()
print("print pose", currentpose)
# grasp 2
# joint1: -0.05215534567832947
# joint2: -0.5307573676109314
# joint3: 0.3911651074886322
# joint4: -0.05522330850362778
# joint5: -0.6565437912940979
# joint6: 0
target2 = [-0.-0.026, -0.50, 0.443, -0.0028, -0.697, 0]
# target = [0.37582528591156006, -0.70, 0.6565437912940979, 0.6120583415031433, -1.193437099456787, -2.356194496154785]
controller.move_to_joint_state(target2)
# controller.move_to_joint_state(target)
# grasp 1
# joint1: 0.37582528591156006
# joint2: -0.7593204975128174
# joint3: 0.6565437912940979
# joint4: 0.6120583415031433
# joint5: -1.193437099456787
# joint6: -2.356194496154785
# gripper: 0.016388067975640297

# grasp 2
# joint1: -0.05215534567832947
# joint2: -0.5307573676109314
# joint3: 0.3911651074886322
# joint4: -0.05522330850362778
# joint5: -0.6565437912940979
# joint6: 0
# gripper: 0.014339559711515903
# time: 0.0
