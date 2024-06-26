cmake_minimum_required(VERSION 3.0.2)
project(end_effector_pose)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning_interface
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp moveit_core moveit_ros_planning_interface
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}
  src/end_effector_pose.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)