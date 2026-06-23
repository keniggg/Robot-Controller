cmake_minimum_required(VERSION 3.0.2)
project(alicia_duo_driver)

# 使用C++11标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
  message_generation
)

# 添加自定义消息
add_message_files(
  FILES
  ArmJointState.msg
)

# 生成消息依赖
generate_messages(
  DEPENDENCIES
  std_msgs
)

# 声明catkin包
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs serial message_runtime
)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 构建库
add_library(${PROJECT_NAME}
  src/serial_server_node.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# 构建可执行文件
add_executable(serial_server_node_exec src/serial_server_node_main.cpp)
set_target_properties(serial_server_node_exec PROPERTIES OUTPUT_NAME serial_server_node)
target_link_libraries(serial_server_node_exec
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# 安装目标
install(TARGETS ${PROJECT_NAME} serial_server_node_exec
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# 安装Python节点
catkin_install_python(PROGRAMS
  scripts/serial_data_type_node.py
  scripts/servo_states_node.py
  scripts/servo_control_node.py
  scripts/arm_control_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装launch文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)