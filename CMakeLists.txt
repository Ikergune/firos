cmake_minimum_required(VERSION 2.8.3)
project(firos)

## Requiered dependencies for firos
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
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
