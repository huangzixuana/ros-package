cmake_minimum_required(VERSION 2.8.12)
project(serial_com)

# Set c++11 cmake flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

add_definitions(-DOPENMP)


# Find rquired packeges

find_package(Boost REQUIRED COMPONENTS thread)
find_package(catkin  REQUIRED
  COMPONENTS
    roscpp
    rospy
    std_msgs
    serial
    sensor_msgs   
)

add_definitions(-O4 -g)


# add_message_files(
#   DIRECTORY msg
#   FILES
#   rtk.msg
# )

# generate_messages(
#   DEPENDENCIES
#   geometry_msgs
#   std_msgs
# )

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    sensor_msgs
  DEPENDS
    Boost
)

include_directories(
  include/serial_com
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


link_directories(
  #${WORKING_DIR}/src/lib/yaml-0.5/build
)

add_library(${PROJECT_NAME}
    src/serial_base.cpp
)

target_link_libraries(${PROJECT_NAME}
  pthread
  stdc++
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(serial_relay_node
  src/serial_relay.cpp
)
target_link_libraries(serial_relay_node
  ${PROJECT_NAME}
)

add_executable(serial_ultrasonic_node
  src/serial_ultrasonic.cpp
)

target_link_libraries(serial_ultrasonic_node
  ${PROJECT_NAME}
)


add_executable(serial_battery_node
  src/serial_battery.cpp
)

target_link_libraries(serial_battery_node
  ${PROJECT_NAME}
)

#add_dependencies(${PROJECT_NAME}_lib
#)
# install(DIRECTORY
#     launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# install(PROGRAMS
#     #scripts/tf_remove_frames.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(TARGETS  serial_usb_encoder_node ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(TARGETS  serial_usb_distance_node ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


