cmake_minimum_required(VERSION 2.8.3)
project(gazebo_camera_demo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  roscpp
  sensor_msgs
  tf
)

catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
