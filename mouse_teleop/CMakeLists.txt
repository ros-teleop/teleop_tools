cmake_minimum_required(VERSION 2.8.3)
project(mouse_teleop)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS scripts/mouse_teleop.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

foreach(dir config launch)
  install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
