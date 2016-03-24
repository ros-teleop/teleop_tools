cmake_minimum_required(VERSION 2.8.3)
project(joy_teleop)

find_package(catkin REQUIRED COMPONENTS)
catkin_package(
  CATKIN_DEPENDS actionlib)

install(PROGRAMS scripts/joy_teleop.py 
                 scripts/incrementer_server.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

foreach(dir config launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
