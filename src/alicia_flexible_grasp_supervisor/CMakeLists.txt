cmake_minimum_required(VERSION 3.0.2)
project(alicia_flexible_grasp_supervisor)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  control_msgs
  controller_manager_msgs
  message_generation
  tf
  tf2_ros
  cv_bridge
  image_transport
)

catkin_python_setup()

add_message_files(
  FILES
  TactileFrame.msg
  TactileState.msg
  ObjectPose.msg
  GraspState.msg
  SafetyState.msg
)

add_service_files(
  FILES
  StartGrasp.srv
  StopGrasp.srv
  SetForceParams.srv
  SetTargetPose.srv
  SetJointCommand.srv
  SetFloat.srv
  CartesianJog.srv
  TriggerZero.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    trajectory_msgs
    control_msgs
    controller_manager_msgs
    message_runtime
    tf
    tf2_ros
    cv_bridge
    image_transport
)

catkin_install_python(PROGRAMS
  scripts/tactile_skin_node.py
  scripts/camera_node.py
  scripts/perception_node.py
  scripts/handeye_transform_node.py
  scripts/motion_gateway_node.py
  scripts/grasp6d_node.py
  scripts/compliant_gripper_controller.py
  scripts/grasp_task_node.py
  scripts/safety_monitor_node.py
  scripts/data_logger_node.py
  scripts/system_bringup_check.py
  gui/main_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config docs gui rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
