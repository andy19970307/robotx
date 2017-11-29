execute_process(COMMAND "/home/david/robotx/catkin_ws/build/vehicle_follow/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/david/robotx/catkin_ws/build/vehicle_follow/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
