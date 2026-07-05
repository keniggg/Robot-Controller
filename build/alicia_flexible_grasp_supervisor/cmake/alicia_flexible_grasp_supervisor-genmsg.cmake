# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "alicia_flexible_grasp_supervisor: 5 messages, 8 services")

set(MSG_I_FLAGS "-Ialicia_flexible_grasp_supervisor:/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(alicia_flexible_grasp_supervisor_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileFrame.msg" NAME_WE)
add_custom_target(_alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "alicia_flexible_grasp_supervisor" "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileFrame.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileState.msg" NAME_WE)
add_custom_target(_alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "alicia_flexible_grasp_supervisor" "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileState.msg" "alicia_flexible_grasp_supervisor/TactileFrame:std_msgs/Header"
)

get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/ObjectPose.msg" NAME_WE)
add_custom_target(_alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "alicia_flexible_grasp_supervisor" "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/ObjectPose.msg" "geometry_msgs/Point:std_msgs/Header:geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/PoseStamped"
)

get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/GraspState.msg" NAME_WE)
add_custom_target(_alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "alicia_flexible_grasp_supervisor" "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/GraspState.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/SafetyState.msg" NAME_WE)
add_custom_target(_alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "alicia_flexible_grasp_supervisor" "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/SafetyState.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/StartGrasp.srv" NAME_WE)
add_custom_target(_alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "alicia_flexible_grasp_supervisor" "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/StartGrasp.srv" ""
)

get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/StopGrasp.srv" NAME_WE)
add_custom_target(_alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "alicia_flexible_grasp_supervisor" "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/StopGrasp.srv" ""
)

get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetForceParams.srv" NAME_WE)
add_custom_target(_alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "alicia_flexible_grasp_supervisor" "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetForceParams.srv" ""
)

get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetTargetPose.srv" NAME_WE)
add_custom_target(_alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "alicia_flexible_grasp_supervisor" "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetTargetPose.srv" "geometry_msgs/Point:std_msgs/Header:geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/PoseStamped"
)

get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetJointCommand.srv" NAME_WE)
add_custom_target(_alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "alicia_flexible_grasp_supervisor" "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetJointCommand.srv" ""
)

get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetFloat.srv" NAME_WE)
add_custom_target(_alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "alicia_flexible_grasp_supervisor" "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetFloat.srv" ""
)

get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/CartesianJog.srv" NAME_WE)
add_custom_target(_alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "alicia_flexible_grasp_supervisor" "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/CartesianJog.srv" ""
)

get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/TriggerZero.srv" NAME_WE)
add_custom_target(_alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "alicia_flexible_grasp_supervisor" "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/TriggerZero.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileFrame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_msg_cpp(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileState.msg"
  "${MSG_I_FLAGS}"
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileFrame.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_msg_cpp(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/ObjectPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_msg_cpp(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/GraspState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_msg_cpp(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/SafetyState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)

### Generating Services
_generate_srv_cpp(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/StartGrasp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_cpp(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/StopGrasp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_cpp(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetForceParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_cpp(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetTargetPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_cpp(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetJointCommand.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_cpp(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetFloat.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_cpp(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/CartesianJog.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_cpp(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/TriggerZero.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)

### Generating Module File
_generate_module_cpp(alicia_flexible_grasp_supervisor
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/alicia_flexible_grasp_supervisor
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(alicia_flexible_grasp_supervisor_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages alicia_flexible_grasp_supervisor_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileFrame.msg" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_cpp _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileState.msg" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_cpp _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/ObjectPose.msg" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_cpp _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/GraspState.msg" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_cpp _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/SafetyState.msg" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_cpp _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/StartGrasp.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_cpp _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/StopGrasp.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_cpp _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetForceParams.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_cpp _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetTargetPose.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_cpp _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetJointCommand.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_cpp _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetFloat.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_cpp _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/CartesianJog.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_cpp _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/TriggerZero.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_cpp _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(alicia_flexible_grasp_supervisor_gencpp)
add_dependencies(alicia_flexible_grasp_supervisor_gencpp alicia_flexible_grasp_supervisor_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS alicia_flexible_grasp_supervisor_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileFrame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_msg_eus(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileState.msg"
  "${MSG_I_FLAGS}"
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileFrame.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_msg_eus(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/ObjectPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_msg_eus(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/GraspState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_msg_eus(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/SafetyState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)

### Generating Services
_generate_srv_eus(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/StartGrasp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_eus(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/StopGrasp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_eus(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetForceParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_eus(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetTargetPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_eus(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetJointCommand.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_eus(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetFloat.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_eus(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/CartesianJog.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_eus(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/TriggerZero.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)

### Generating Module File
_generate_module_eus(alicia_flexible_grasp_supervisor
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/alicia_flexible_grasp_supervisor
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(alicia_flexible_grasp_supervisor_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages alicia_flexible_grasp_supervisor_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileFrame.msg" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_eus _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileState.msg" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_eus _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/ObjectPose.msg" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_eus _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/GraspState.msg" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_eus _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/SafetyState.msg" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_eus _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/StartGrasp.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_eus _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/StopGrasp.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_eus _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetForceParams.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_eus _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetTargetPose.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_eus _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetJointCommand.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_eus _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetFloat.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_eus _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/CartesianJog.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_eus _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/TriggerZero.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_eus _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(alicia_flexible_grasp_supervisor_geneus)
add_dependencies(alicia_flexible_grasp_supervisor_geneus alicia_flexible_grasp_supervisor_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS alicia_flexible_grasp_supervisor_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileFrame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_msg_lisp(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileState.msg"
  "${MSG_I_FLAGS}"
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileFrame.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_msg_lisp(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/ObjectPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_msg_lisp(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/GraspState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_msg_lisp(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/SafetyState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)

### Generating Services
_generate_srv_lisp(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/StartGrasp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_lisp(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/StopGrasp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_lisp(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetForceParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_lisp(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetTargetPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_lisp(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetJointCommand.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_lisp(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetFloat.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_lisp(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/CartesianJog.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_lisp(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/TriggerZero.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)

### Generating Module File
_generate_module_lisp(alicia_flexible_grasp_supervisor
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/alicia_flexible_grasp_supervisor
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(alicia_flexible_grasp_supervisor_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages alicia_flexible_grasp_supervisor_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileFrame.msg" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_lisp _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileState.msg" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_lisp _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/ObjectPose.msg" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_lisp _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/GraspState.msg" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_lisp _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/SafetyState.msg" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_lisp _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/StartGrasp.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_lisp _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/StopGrasp.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_lisp _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetForceParams.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_lisp _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetTargetPose.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_lisp _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetJointCommand.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_lisp _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetFloat.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_lisp _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/CartesianJog.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_lisp _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/TriggerZero.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_lisp _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(alicia_flexible_grasp_supervisor_genlisp)
add_dependencies(alicia_flexible_grasp_supervisor_genlisp alicia_flexible_grasp_supervisor_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS alicia_flexible_grasp_supervisor_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileFrame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_msg_nodejs(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileState.msg"
  "${MSG_I_FLAGS}"
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileFrame.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_msg_nodejs(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/ObjectPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_msg_nodejs(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/GraspState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_msg_nodejs(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/SafetyState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)

### Generating Services
_generate_srv_nodejs(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/StartGrasp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_nodejs(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/StopGrasp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_nodejs(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetForceParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_nodejs(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetTargetPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_nodejs(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetJointCommand.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_nodejs(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetFloat.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_nodejs(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/CartesianJog.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_nodejs(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/TriggerZero.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)

### Generating Module File
_generate_module_nodejs(alicia_flexible_grasp_supervisor
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/alicia_flexible_grasp_supervisor
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(alicia_flexible_grasp_supervisor_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages alicia_flexible_grasp_supervisor_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileFrame.msg" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_nodejs _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileState.msg" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_nodejs _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/ObjectPose.msg" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_nodejs _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/GraspState.msg" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_nodejs _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/SafetyState.msg" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_nodejs _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/StartGrasp.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_nodejs _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/StopGrasp.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_nodejs _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetForceParams.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_nodejs _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetTargetPose.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_nodejs _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetJointCommand.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_nodejs _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetFloat.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_nodejs _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/CartesianJog.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_nodejs _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/TriggerZero.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_nodejs _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(alicia_flexible_grasp_supervisor_gennodejs)
add_dependencies(alicia_flexible_grasp_supervisor_gennodejs alicia_flexible_grasp_supervisor_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS alicia_flexible_grasp_supervisor_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileFrame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_msg_py(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileState.msg"
  "${MSG_I_FLAGS}"
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileFrame.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_msg_py(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/ObjectPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_msg_py(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/GraspState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_msg_py(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/SafetyState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)

### Generating Services
_generate_srv_py(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/StartGrasp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_py(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/StopGrasp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_py(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetForceParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_py(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetTargetPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_py(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetJointCommand.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_py(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetFloat.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_py(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/CartesianJog.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)
_generate_srv_py(alicia_flexible_grasp_supervisor
  "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/TriggerZero.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/alicia_flexible_grasp_supervisor
)

### Generating Module File
_generate_module_py(alicia_flexible_grasp_supervisor
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/alicia_flexible_grasp_supervisor
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(alicia_flexible_grasp_supervisor_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages alicia_flexible_grasp_supervisor_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileFrame.msg" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_py _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileState.msg" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_py _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/ObjectPose.msg" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_py _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/GraspState.msg" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_py _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/SafetyState.msg" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_py _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/StartGrasp.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_py _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/StopGrasp.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_py _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetForceParams.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_py _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetTargetPose.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_py _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetJointCommand.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_py _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetFloat.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_py _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/CartesianJog.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_py _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/TriggerZero.srv" NAME_WE)
add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_py _alicia_flexible_grasp_supervisor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(alicia_flexible_grasp_supervisor_genpy)
add_dependencies(alicia_flexible_grasp_supervisor_genpy alicia_flexible_grasp_supervisor_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS alicia_flexible_grasp_supervisor_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/alicia_flexible_grasp_supervisor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/alicia_flexible_grasp_supervisor
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/alicia_flexible_grasp_supervisor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/alicia_flexible_grasp_supervisor
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/alicia_flexible_grasp_supervisor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/alicia_flexible_grasp_supervisor
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/alicia_flexible_grasp_supervisor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/alicia_flexible_grasp_supervisor
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/alicia_flexible_grasp_supervisor)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/alicia_flexible_grasp_supervisor\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/alicia_flexible_grasp_supervisor
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(alicia_flexible_grasp_supervisor_generate_messages_py geometry_msgs_generate_messages_py)
endif()
