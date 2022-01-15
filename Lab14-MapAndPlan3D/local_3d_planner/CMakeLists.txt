cmake_minimum_required(VERSION 2.8.3)
project(planner_3d)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  pcl_conversions
  message_generation
  message_runtime
  tf2
  tf2_ros
  tf2_msgs
  tf2_sensor_msgs
  octomap_msgs
  sensor_msgs
  interactive_markers
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
find_package(ompl REQUIRED)
find_package(fcl REQUIRED)


catkin_package(
        CATKIN_DEPENDS roscpp rospy std_msgs pcl_conversions message_runtime tf2 tf2_ros tf2_msgs tf2_sensor_msgs interactive_markers
        DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OCTOMAP_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${fcl_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
        src/ValidityChecker.cpp
        src/planningObjectives/BiasedRealVectorStateSpace.cpp
        src/planningObjectives/PlanningObjectives.cpp
        src/Planner3D.cpp
        src/Planner3DNode.cpp)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   fcl
   ${OCTOMAP_LIBRARIES}
   ${OMPL_LIBRARIES}
)