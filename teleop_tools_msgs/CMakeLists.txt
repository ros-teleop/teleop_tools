cmake_minimum_required(VERSION 2.8.3)
project(teleop_tools_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  control_msgs
  message_generation)

add_action_files(DIRECTORY action FILES Increment.action)
generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime 
                 actionlib_msgs
)

