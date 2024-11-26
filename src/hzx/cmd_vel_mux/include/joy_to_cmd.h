/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-03-13 14:30:58
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2023-03-21 16:52:56
 * @FilePath: /can_package/src/cmd_vel_mux/include/cmd_vel_mux.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#pragma once
#ifndef _JOY_TO_CMD_H_
#define _JOY_TO_CMD_H_

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include<std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Bool.h>
#include "cmd_node.h"
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

namespace joy_to_cmd {

enum ControlType {
    JOY_IDLE = 0,
    JOY_ENBLE ,
    JOY_SPEED ,
    JOY_ARM ,
    JOY_STOP
};

class JoyToCmd {

  public:
    JoyToCmd(ros::NodeHandle &nhandle,
             std::string name,
             float linear,
             float angular);
    ~JoyToCmd() {}

    void JoyHandler(const sensor_msgs::JoyConstPtr msg);
    void TimeHandler(const ros::TimerEvent &event);
    void Estop_sub(const std_msgs::Bool::ConstPtr& msg);

    void plcSub(const diagnostic_msgs::DiagnosticArray::ConstPtr & msg);

    int Sign0(float input) {
        if (input > 0)return 1;
        if (input < 0)return -1;
        return 0;
    }

  public:
    ros::Subscriber sub_joy;
    ros::Subscriber estop_sub;

    // plc24
    ros::Subscriber plc_sub;

    ros::Publisher pub_cmd_vel;
    ros::Publisher pub_arm_vel;
    ros::Publisher pub_diag;
    ros::Publisher pub_arm_cancel_goal;//zs add
    ros::Publisher pub_plc_;//wl add


  private:
    ros::NodeHandle nh;
    ros::Timer timer;
    std::string topic_name;
    volatile ControlType joy_type = JOY_IDLE;   //手柄归位标志，默认为归位
    ControlType last_joy_type = JOY_IDLE;
    geometry_msgs::Twist cmd;
    sensor_msgs::Joy cmd_joy;
    float max_linear_speed;
    float max_angular_speed;
    float linear_speed;
    float angular_speed;
    bool stop_state = false;
    bool stop_button = false;
    bool last_stop_button = false;
    ros::Time emergence_time;
    ros::Time joy_sub_time;

    bool cap_on_status_=false;
    bool last_stop_on_=false;
    
};
}

#endif
