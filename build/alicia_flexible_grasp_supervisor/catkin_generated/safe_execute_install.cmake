execute_process(COMMAND "/home/zhuyupei/alicia_wa_full/build/alicia_flexible_grasp_supervisor/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/zhuyupei/alicia_wa_full/build/alicia_flexible_grasp_supervisor/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
