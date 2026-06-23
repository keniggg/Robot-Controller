cmake_minimum_required(VERSION 3.0.2)
project(alicia_duo_ros_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf
  message_generation  # Add message_generation for custom messages
  alicia_duo_driver  # Add the dependency for serial_server_node
)

## Generate messages
generate_messages(
  DEPENDENCIES
  std_msgs
  alicia_duo_driver  # Ensure serial_server_node is included for message generation
)

## Declare the package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime alicia_duo_driver  # message_runtime for custom message generation
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# =====================
# Serial Communication Helper Library
# =====================
add_library(${PROJECT_NAME}_serial_comm_helper
  src/serial_comm_helper.cpp
)
add_dependencies(${PROJECT_NAME}_serial_comm_helper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_serial_comm_helper ${catkin_LIBRARIES})

# =====================
# Test Serial Read Node
# =====================
add_executable(test_serial_read_node src/test_serial_read_main.cpp)
add_dependencies(test_serial_read_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_serial_read_node
  ${PROJECT_NAME}_serial_comm_helper
  ${catkin_LIBRARIES}
)

# =====================
# Alicia Hardware Interface Library
# =====================
add_library(alicia_hw
  src/alicia_hw.cpp
)
add_dependencies(alicia_hw ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(alicia_hw ${catkin_LIBRARIES})

# =====================
# Hardware Interface Node (Main Entry Point)
# =====================
add_executable(hardware_interface_node src/hardware_interface_node.cpp)
target_link_libraries(hardware_interface_node
  alicia_hw
  ${PROJECT_NAME}_serial_comm_helper     
  ${catkin_LIBRARIES}
)
add_dependencies(hardware_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
