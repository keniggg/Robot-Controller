# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "alicia_duo_driver: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ialicia_duo_driver:/home/zhuyupei/alicia_wa_full/src/real-arm/alicia_duo_driver/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(alicia_duo_driver_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/real-arm/alicia_duo_driver/msg/ArmJointState.msg" NAME_WE)
add_custom_target(_alicia_duo_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "alicia_duo_driver" "/home/zhuyupei/alicia_wa_full/src/real-arm/alicia_duo_driver/msg/ArmJointState.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(alicia_duo_driver
  "/home/zhuyupei/alicia_wa_full/src/real-arm/alicia_duo_driver/msg/ArmJointState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/alicia_duo_driver
)

### Generating Services

### Generating Module File
_generate_module_cpp(alicia_duo_driver
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/alicia_duo_driver
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(alicia_duo_driver_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(alicia_duo_driver_generate_messages alicia_duo_driver_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/real-arm/alicia_duo_driver/msg/ArmJointState.msg" NAME_WE)
add_dependencies(alicia_duo_driver_generate_messages_cpp _alicia_duo_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(alicia_duo_driver_gencpp)
add_dependencies(alicia_duo_driver_gencpp alicia_duo_driver_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS alicia_duo_driver_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(alicia_duo_driver
  "/home/zhuyupei/alicia_wa_full/src/real-arm/alicia_duo_driver/msg/ArmJointState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/alicia_duo_driver
)

### Generating Services

### Generating Module File
_generate_module_eus(alicia_duo_driver
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/alicia_duo_driver
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(alicia_duo_driver_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(alicia_duo_driver_generate_messages alicia_duo_driver_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/real-arm/alicia_duo_driver/msg/ArmJointState.msg" NAME_WE)
add_dependencies(alicia_duo_driver_generate_messages_eus _alicia_duo_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(alicia_duo_driver_geneus)
add_dependencies(alicia_duo_driver_geneus alicia_duo_driver_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS alicia_duo_driver_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(alicia_duo_driver
  "/home/zhuyupei/alicia_wa_full/src/real-arm/alicia_duo_driver/msg/ArmJointState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/alicia_duo_driver
)

### Generating Services

### Generating Module File
_generate_module_lisp(alicia_duo_driver
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/alicia_duo_driver
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(alicia_duo_driver_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(alicia_duo_driver_generate_messages alicia_duo_driver_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/real-arm/alicia_duo_driver/msg/ArmJointState.msg" NAME_WE)
add_dependencies(alicia_duo_driver_generate_messages_lisp _alicia_duo_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(alicia_duo_driver_genlisp)
add_dependencies(alicia_duo_driver_genlisp alicia_duo_driver_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS alicia_duo_driver_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(alicia_duo_driver
  "/home/zhuyupei/alicia_wa_full/src/real-arm/alicia_duo_driver/msg/ArmJointState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/alicia_duo_driver
)

### Generating Services

### Generating Module File
_generate_module_nodejs(alicia_duo_driver
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/alicia_duo_driver
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(alicia_duo_driver_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(alicia_duo_driver_generate_messages alicia_duo_driver_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/real-arm/alicia_duo_driver/msg/ArmJointState.msg" NAME_WE)
add_dependencies(alicia_duo_driver_generate_messages_nodejs _alicia_duo_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(alicia_duo_driver_gennodejs)
add_dependencies(alicia_duo_driver_gennodejs alicia_duo_driver_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS alicia_duo_driver_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(alicia_duo_driver
  "/home/zhuyupei/alicia_wa_full/src/real-arm/alicia_duo_driver/msg/ArmJointState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/alicia_duo_driver
)

### Generating Services

### Generating Module File
_generate_module_py(alicia_duo_driver
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/alicia_duo_driver
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(alicia_duo_driver_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(alicia_duo_driver_generate_messages alicia_duo_driver_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zhuyupei/alicia_wa_full/src/real-arm/alicia_duo_driver/msg/ArmJointState.msg" NAME_WE)
add_dependencies(alicia_duo_driver_generate_messages_py _alicia_duo_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(alicia_duo_driver_genpy)
add_dependencies(alicia_duo_driver_genpy alicia_duo_driver_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS alicia_duo_driver_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/alicia_duo_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/alicia_duo_driver
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(alicia_duo_driver_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/alicia_duo_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/alicia_duo_driver
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(alicia_duo_driver_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/alicia_duo_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/alicia_duo_driver
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(alicia_duo_driver_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/alicia_duo_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/alicia_duo_driver
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(alicia_duo_driver_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/alicia_duo_driver)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/alicia_duo_driver\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/alicia_duo_driver
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(alicia_duo_driver_generate_messages_py std_msgs_generate_messages_py)
endif()
