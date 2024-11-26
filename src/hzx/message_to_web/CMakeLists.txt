cmake_minimum_required(VERSION 3.0.2)
project(message_to_web)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  diagnostic_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp diagnostic_msgs
)

## Declare a C++ executable
add_executable(message_to_web src/message_to_web.cpp)

## Link libraries
target_link_libraries(message_to_web ${catkin_LIBRARIES})

## Install rules
install(TARGETS message_to_web
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
