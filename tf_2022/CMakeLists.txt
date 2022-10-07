cmake_minimum_required(VERSION 3.0.2)
project(tf_2022)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
)

catkin_package(
   CATKIN_DEPENDS geometry_msgs rospy std_msgs
)
