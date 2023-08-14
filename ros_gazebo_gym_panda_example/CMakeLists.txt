cmake_minimum_required(VERSION 3.5)
project(ros_gazebo_gym_panda_example)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  ros_gazebo_gym
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_gazebo_gym_panda_example
#  CATKIN_DEPENDS ros_gazebo_gym rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
