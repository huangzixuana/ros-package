cmake_minimum_required(VERSION 3.0.2)
project(imu_acceleration_monitor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_acceleration_monitor src/imu_acceleration_monitor.cpp)
target_link_libraries(imu_acceleration_monitor ${catkin_LIBRARIES})