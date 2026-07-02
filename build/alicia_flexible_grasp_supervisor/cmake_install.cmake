# Install script for directory: /home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/zhuyupei/alicia_wa_full/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/zhuyupei/alicia_wa_full/build/alicia_flexible_grasp_supervisor/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/alicia_flexible_grasp_supervisor/msg" TYPE FILE FILES
    "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileFrame.msg"
    "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/TactileState.msg"
    "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/ObjectPose.msg"
    "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/GraspState.msg"
    "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/msg/SafetyState.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/alicia_flexible_grasp_supervisor/srv" TYPE FILE FILES
    "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/StartGrasp.srv"
    "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/StopGrasp.srv"
    "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetForceParams.srv"
    "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetTargetPose.srv"
    "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetJointCommand.srv"
    "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/SetFloat.srv"
    "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/CartesianJog.srv"
    "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/srv/TriggerZero.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/alicia_flexible_grasp_supervisor/cmake" TYPE FILE FILES "/home/zhuyupei/alicia_wa_full/build/alicia_flexible_grasp_supervisor/catkin_generated/installspace/alicia_flexible_grasp_supervisor-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/zhuyupei/alicia_wa_full/devel/include/alicia_flexible_grasp_supervisor")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/zhuyupei/alicia_wa_full/devel/share/roseus/ros/alicia_flexible_grasp_supervisor")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/zhuyupei/alicia_wa_full/devel/share/common-lisp/ros/alicia_flexible_grasp_supervisor")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/zhuyupei/alicia_wa_full/devel/share/gennodejs/ros/alicia_flexible_grasp_supervisor")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/zhuyupei/alicia_wa_full/devel/lib/python3/dist-packages/alicia_flexible_grasp_supervisor")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/zhuyupei/alicia_wa_full/devel/lib/python3/dist-packages/alicia_flexible_grasp_supervisor")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/zhuyupei/alicia_wa_full/build/alicia_flexible_grasp_supervisor/catkin_generated/installspace/alicia_flexible_grasp_supervisor.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/alicia_flexible_grasp_supervisor/cmake" TYPE FILE FILES "/home/zhuyupei/alicia_wa_full/build/alicia_flexible_grasp_supervisor/catkin_generated/installspace/alicia_flexible_grasp_supervisor-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/alicia_flexible_grasp_supervisor/cmake" TYPE FILE FILES
    "/home/zhuyupei/alicia_wa_full/build/alicia_flexible_grasp_supervisor/catkin_generated/installspace/alicia_flexible_grasp_supervisorConfig.cmake"
    "/home/zhuyupei/alicia_wa_full/build/alicia_flexible_grasp_supervisor/catkin_generated/installspace/alicia_flexible_grasp_supervisorConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/alicia_flexible_grasp_supervisor" TYPE FILE FILES "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/alicia_flexible_grasp_supervisor" TYPE PROGRAM FILES "/home/zhuyupei/alicia_wa_full/build/alicia_flexible_grasp_supervisor/catkin_generated/installspace/tactile_skin_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/alicia_flexible_grasp_supervisor" TYPE PROGRAM FILES "/home/zhuyupei/alicia_wa_full/build/alicia_flexible_grasp_supervisor/catkin_generated/installspace/camera_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/alicia_flexible_grasp_supervisor" TYPE PROGRAM FILES "/home/zhuyupei/alicia_wa_full/build/alicia_flexible_grasp_supervisor/catkin_generated/installspace/perception_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/alicia_flexible_grasp_supervisor" TYPE PROGRAM FILES "/home/zhuyupei/alicia_wa_full/build/alicia_flexible_grasp_supervisor/catkin_generated/installspace/handeye_transform_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/alicia_flexible_grasp_supervisor" TYPE PROGRAM FILES "/home/zhuyupei/alicia_wa_full/build/alicia_flexible_grasp_supervisor/catkin_generated/installspace/motion_gateway_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/alicia_flexible_grasp_supervisor" TYPE PROGRAM FILES "/home/zhuyupei/alicia_wa_full/build/alicia_flexible_grasp_supervisor/catkin_generated/installspace/grasp6d_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/alicia_flexible_grasp_supervisor" TYPE PROGRAM FILES "/home/zhuyupei/alicia_wa_full/build/alicia_flexible_grasp_supervisor/catkin_generated/installspace/compliant_gripper_controller.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/alicia_flexible_grasp_supervisor" TYPE PROGRAM FILES "/home/zhuyupei/alicia_wa_full/build/alicia_flexible_grasp_supervisor/catkin_generated/installspace/grasp_task_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/alicia_flexible_grasp_supervisor" TYPE PROGRAM FILES "/home/zhuyupei/alicia_wa_full/build/alicia_flexible_grasp_supervisor/catkin_generated/installspace/safety_monitor_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/alicia_flexible_grasp_supervisor" TYPE PROGRAM FILES "/home/zhuyupei/alicia_wa_full/build/alicia_flexible_grasp_supervisor/catkin_generated/installspace/data_logger_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/alicia_flexible_grasp_supervisor" TYPE PROGRAM FILES "/home/zhuyupei/alicia_wa_full/build/alicia_flexible_grasp_supervisor/catkin_generated/installspace/system_bringup_check.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/alicia_flexible_grasp_supervisor" TYPE PROGRAM FILES "/home/zhuyupei/alicia_wa_full/build/alicia_flexible_grasp_supervisor/catkin_generated/installspace/main_gui.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/alicia_flexible_grasp_supervisor" TYPE DIRECTORY FILES
    "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/launch"
    "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/config"
    "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/docs"
    "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/gui"
    "/home/zhuyupei/alicia_wa_full/src/alicia_flexible_grasp_supervisor/rviz"
    )
endif()

