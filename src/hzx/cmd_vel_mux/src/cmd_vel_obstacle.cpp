/******************************************************************************
 * Copyright 2020-2025, zhangsai. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES O
    void joyHandler(const std_msgs::HeaderConstPtr& msg) {
        mode_joy = msg->seq;//1 急停按下, 2手动, 3自动
        joy_receive_time = ros::Time::now();R CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Header.h>

class Cmd_Vel_Obstacle {

  private:

    ros::Subscriber subcmd_mux_fusion, subcmd_stop, sub_joy_status, sub_arm_rotate;
    ros::Publisher pubcmd_result;
    ros::Time cmd_vel_obstacle_time, joy_receive_time, arm_rotating_time;
    bool cmd_vel_obstacle_init;
    geometry_msgs::Twist cmd_vel_obstacle_spd;
    int mode_joy;
    bool arm_rotating;

  public:
    Cmd_Vel_Obstacle(ros::NodeHandle nh_) {
        ros::NodeHandle nh_param("~");
        std::string cmd_mux_fusion_topic, cmd_vel_obstacle_topic, cmd_vel_relsult_topic;
        nh_param.param<std::string>("cmd_mux_fusion_topic", cmd_mux_fusion_topic, "/cmd_vel_mux_rectified");
        nh_param.param<std::string>("cmd_vel_obstacle_topic", cmd_vel_obstacle_topic, "/cmd_vel_stop");
        nh_param.param<std::string>("cmd_vel_relsult_topic", cmd_vel_relsult_topic, "/cmd_vel_rectified");

        init();
        subcmd_mux_fusion = nh_.subscribe<geometry_msgs::Twist>(cmd_mux_fusion_topic.c_str(), 5, &Cmd_Vel_Obstacle::CmdMuxFusionHandler, this);
        subcmd_stop = nh_.subscribe<geometry_msgs::Twist>(cmd_vel_obstacle_topic.c_str(), 5, &Cmd_Vel_Obstacle::CmdStopHandler, this);

        pubcmd_result = nh_.advertise<geometry_msgs::Twist>(cmd_vel_relsult_topic.c_str(), 10);
        sub_joy_status = nh_.subscribe<std_msgs::Header>("/joy_status", 1, &Cmd_Vel_Obstacle::joyHandler, this);
        sub_arm_rotate = nh_.subscribe<std_msgs::Header>("/arm_rotate_trig", 1, &Cmd_Vel_Obstacle::arm_rotate_Handler, this);
    }
    ~Cmd_Vel_Obstacle() {}

    void init() {
        cmd_vel_obstacle_init = false;
        arm_rotating = false;
    }

    void joyHandler(const std_msgs::HeaderConstPtr& msg) {
        mode_joy = msg->seq;//1 急停按下, 2手动, 3自动
        joy_receive_time = ros::Time::now();
    }

    void arm_rotate_Handler(const std_msgs::HeaderConstPtr& msg) {
        if (msg->seq == 0) {
            arm_rotating = true;
            arm_rotating_time = ros::Time::now();
        } else {
            arm_rotating = false;
        }
    }

    void CmdMuxFusionHandler(const geometry_msgs::TwistConstPtr& msg) {

        geometry_msgs::Twist twist_rel;
        twist_rel = *msg;
        if (ros::Time::now() - joy_receive_time < ros::Duration(1.0)) {
            if ((mode_joy == 3) && cmd_vel_obstacle_init && (ros::Time::now() - cmd_vel_obstacle_time < ros::Duration(1.0))) {
                if (fabs(msg->linear.x) > cmd_vel_obstacle_spd.linear.x) {
                    twist_rel = cmd_vel_obstacle_spd;
                    if (msg->linear.x < 0) {
                        twist_rel.linear.x = -cmd_vel_obstacle_spd.linear.x;
                    }
                }
            }
            if (mode_joy == 1) {
                twist_rel.linear.x = 0;
                twist_rel.linear.y = 0;
                twist_rel.linear.z = 0;
                twist_rel.angular.x = 0;
                twist_rel.angular.y = 0;
                twist_rel.angular.z = 0;
            }
            if(mode_joy == 3){
            if (arm_rotating == true) {
                twist_rel.linear.x = 0;
                twist_rel.linear.y = 0;
                twist_rel.linear.z = 0;
                twist_rel.angular.x = 0;
                twist_rel.angular.y = 0;
                twist_rel.angular.z = 0;
                if (ros::Time::now() - arm_rotating_time > ros::Duration(50.0)){
                    arm_rotating = false;
                }
            }
            }else{
                arm_rotating = false;
            }
        }
        pubcmd_result.publish(twist_rel);
    }
    void CmdStopHandler(const geometry_msgs::TwistConstPtr& msg) {
        cmd_vel_obstacle_init = true;
        cmd_vel_obstacle_time = ros::Time::now();
        cmd_vel_obstacle_spd = *msg;
    }

};


int main(int argc, char **argv) {
    ros::init(argc, argv, "cmd_vel_obstacle");
    ros::NodeHandle NodeHandle;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    std::unique_ptr<Cmd_Vel_Obstacle> cmd_vel_obstacle_(new Cmd_Vel_Obstacle(NodeHandle));
    ros::waitForShutdown();
    return 0;
}