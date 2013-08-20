cmake_minimum_required(VERSION 2.4.6)

if(USE_ROSBUILD)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

  rosbuild_init()

else()

  find_package(catkin REQUIRED COMPONENTS rospy play_motion actionlib)

  catkin_package()

  install(PROGRAMS scripts/joy_teleop.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

endif()
