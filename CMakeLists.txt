cmake_minimum_required(VERSION 2.8.3)
project(firos)

## Requiered dependencies for firos
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Generate messages in the 'data/ros_msgs' folder
add_message_files(
  DIRECTORY 
  data/ros_msgs
  FILES
  Robot_Event.msg
  CB_Event.msg
)

## Generate services in the 'data/ros_service_info' folder
add_service_files(
  DIRECTORY 
  data/ros_service_info
  FILES
  FIROS_Info.srv
)

## Generate messages which firos depends on
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Included directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)
