cmake_minimum_required(VERSION 3.0.2)
project(mouse_teleop)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS scripts/mouse_teleop.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

foreach(dir config launch)
  install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
