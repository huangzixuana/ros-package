cmake_minimum_required(VERSION 3.0.2)
project(joy_to_ackermann)

## 找到 ROS 依赖项并进行包含
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  ackermann_msgs
  roslib
)

## 找到 yaml-cpp 的系统安装
find_package(yaml-cpp REQUIRED)

## 添加 catkin 中的依赖库文件
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs ackermann_msgs
)

## 设定包含目录
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

## 设定可执行文件
add_executable(${PROJECT_NAME} src/joy_to_ackermann.cpp)

## 链接所需的库文件
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)


## 指定安装目录
install(TARGETS joy_to_ackermann
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
