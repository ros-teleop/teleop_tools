cmake_minimum_required(VERSION 3.0.2)
project(key_teleop)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS scripts/key_teleop.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
