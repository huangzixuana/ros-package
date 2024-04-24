cmake_minimum_required(VERSION 3.0.2)
project(pose_transformer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  roslib
)


find_package(yaml-cpp REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/pose_transformer.cpp
# )

add_executable(${PROJECT_NAME}
  src/pose_transformer.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)