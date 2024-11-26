/*
 * @Author: hambin
 * @Date: 2023-03-10 13:40:12
 * @LastEditors: Please set LastEditors
 * @FilePath: src/cmd_vel_mux.cpp
 * @Description: can to joy message
 */
#include "joy_to_cmd.h"
#include <signal.h>
namespace joy_to_cmd {
JoyToCmd::JoyToCmd(ros::NodeHandle& nhandle, std::string name, float linear, float angular) {
    nh = nhandle;
    topic_name = name;
    max_linear_speed = linear;
    max_angular_speed = angular;
    linear_speed = max_linear_speed;    //* 0.5;
    angular_speed = max_angular_speed;  //* 0.5;
    // 订阅控制器话题
    sub_joy = nh.subscribe("joy", 1, &JoyToCmd::JoyHandler, this);
    
    estop_sub = nh.subscribe("/estop",1, &JoyToCmd::Estop_sub, this);
    pub_diag = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);
    // 发布控制器速度命令
    pub_cmd_vel = nh.advertise<geometry_msgs::Twist>(topic_name, 1);
    pub_arm_vel = nh.advertise<sensor_msgs::Joy>("joy", 1);

    timer = nh.createTimer(ros::Duration(0.05), &JoyToCmd::TimeHandler, this);
    // pub_arm_cancel_goal = nh.advertise<trajectory_msgs::JointTrajectory>("/leapting_controller/command", 1);//zs add
    pub_arm_cancel_goal = nh.advertise<std_msgs::Bool>("/flexbe/command/pause", 1);  // zs add

    pub_cup_off_=nh.advertise<std_msgs::String>("/plc24_request", 1); //wl add
}
void JoyToCmd::Estop_sub(const std_msgs::Bool::ConstPtr& msg) {
    if(msg->data) {
        ROS_INFO("Emergency stop activated!");
        stop_state = true;
        
    } else {
        ROS_INFO("Emergency stop deactivated.");
        stop_state = false;
    }
}
void JoyToCmd::JoyHandler(const sensor_msgs::JoyConstPtr msg) {
    // 是否是当前节点的joy
    if (msg->header.frame_id != topic_name)
        return;
    // 急停按下zs add
    if (msg->buttons[6] == 1 || stop_state) {
        joy_type = JOY_STOP;
        stop_state = true;

        cmd.linear.z = 1;  // 1急停按下;2手动;3自动
        std::cout << "急停按钮被按下button！" << std::endl;
        // 急停被按下或者未定义对应动作，置零位
        cmd.linear.x = 0;
        cmd.angular.z = 0;
        linear_speed = max_linear_speed;    //* 0.5;
        angular_speed = max_angular_speed;  // * 0.5;
        emergence_time = ros::Time::now();

        cmd_joy.header.stamp = ros::Time::now();
        cmd_joy.header.frame_id = "jtc_" + topic_name;
        cmd_joy.axes.resize(6);
        cmd_joy.axes[0] =  0;
        cmd_joy.axes[1] =  0;
        cmd_joy.axes[2] =  0;
        cmd_joy.axes[3] =  0;
        cmd_joy.axes[4] =  0;
        cmd_joy.axes[5] =  0;
        
        if (msg->buttons[6] == 1) {
            stop_button = true;
        }else{
            stop_button = false;
        }
        if (last_stop_button != stop_button){
            stop_state = false;
        }
        last_stop_button = stop_button;
        return;
    }
    stop_state = false;
    // 判断手柄是否归位
    if ((fabs(msg->axes[2]) == 1 && fabs(msg->axes[5]) == 0) || (fabs(msg->axes[2]) == 0 && fabs(msg->axes[5]) == 1) ||
        (fabs(msg->axes[2]) == 1 && fabs(msg->axes[5]) == 1) || (fabs(msg->axes[2]) == 0 && fabs(msg->axes[5]) == 0)) {
        joy_type = JOY_IDLE;
        cmd.linear.z = 3;  // 1急停按下;2手动;3自动
        return;
    }
    joy_type = JOY_ENBLE;
    cmd.linear.z = 2;  // 1急停按下;2手动;3自动
    // 急停未按下
    if (msg->buttons[6] != 1) {
        // 速度设置
        if (fabs(msg->axes[2]) == 0.5 && fabs(msg->axes[5]) == 0) {  //  && (fabs(msg->axes[1]!=0) || msg->axes[3]))
            // linear_speed += (msg->axes[7] * 0.001);
            // angular_speed += (msg->axes[6] * 0.001);
            cmd.linear.x = msg->axes[1] * linear_speed;
            cmd.angular.z = msg->axes[3] * angular_speed;
            cmd.linear.x = fabs(cmd.linear.x) <= max_linear_speed ? cmd.linear.x : Sign0(cmd.linear.x) * max_linear_speed;
            cmd.angular.z = fabs(cmd.angular.z) <= max_angular_speed ? cmd.angular.z : Sign0(cmd.linear.z) * max_angular_speed;
            // std::cout << "线速度：" << cmd.linear.x << "  角速度：" << cmd.angular.z << std::endl;
            joy_type = JOY_SPEED;
            if (msg->buttons[0] == 1){
                cmd.linear.z = 4;
            }
            else{
                cmd.linear.z = 2;  // 1急停按下;2手动;3自动
            }
            return;
        }
        // 机械臂设置
        if (fabs(msg->axes[5]) == 0.5 && fabs(msg->axes[2] == 0)) {
            // to do
            joy_type = JOY_ARM;
            cmd.linear.z = 2;  // 1急停按下;2手动;3自动
            cmd_joy.header.stamp = ros::Time::now();
            cmd_joy.header.frame_id = "jtc_" + topic_name;
            cmd_joy.axes.resize(6);
            cmd_joy.axes.clear();            if (msg->buttons[4] == 1) {

            if (msg->buttons[0] == 1) {
                // std::cout << "机械臂1动作" << std::endl;
                cmd_joy.axes.resize(6);
                cmd_joy.axes[0] = msg->axes[1] * 0.1;
            }
            else if (msg->buttons[1] == 1) {
                // std::cout << "机械臂2动作" << std::endl;
                cmd_joy.axes.resize(6);
                cmd_joy.axes[1] = msg->axes[1] * 0.1;
            }
            else if (msg->buttons[2] == 1) {
                // std::cout << "机械臂3动作" << std::endl;
                cmd_joy.axes.resize(6);
                cmd_joy.axes[2] = msg->axes[1] * 0.1;
            }
            else if (msg->buttons[3] == 1) {
                // std::cout << "机械臂4动作" << std::endl;
                cmd_joy.axes.resize(6);
                cmd_joy.axes[3] = msg->axes[1] * 0.1;
            }
                // std::cout << "机械臂5动作" << std::endl;
                cmd_joy.axes.resize(6);
                cmd_joy.axes[4] = msg->axes[1] * 0.1;
            }
            else if (msg->buttons[5] == 1) {
                // std::cout << "机械臂6动作" << std::endl;
                cmd_joy.axes.resize(6);
                cmd_joy.axes[5] = msg->axes[1] * 0.1;
            }
            else if (msg->buttons[7] ==1 && cap_on_status_){
                std_msgs::String msg;
                msg.data="cup_off";
                pub_cup_off_.publish(msg);
                cap_on_status_=false;
            }
            return;
        }
    } else {
        joy_type = JOY_STOP;
        cmd.linear.z = 1;  // 1急停按下;2手动;3自动
        std::cout << "急停按钮被按下！" << std::endl;
    }
    // 急停被按下或者未定义对应动作，置零位
    cmd.linear.x = 0;
    cmd.angular.z = 0;
    linear_speed = max_linear_speed;    //* 0.5;
    angular_speed = max_angular_speed;  // * 0.5;
}
void JoyToCmd::TimeHandler(const ros::TimerEvent& event) {
    // //zs_add
    // std_msgs::Header joy_status;
    // if ((ros::Time::now() - emergence_time < ros::Duration(1.0)) || (joy_type == JOY_STOP)) {
    //     joy_status.seq = 1;//急停按下
    // } else if (joy_type == JOY_ENBLE) {
    //     joy_status.seq = 2;//手动
    // } else  if (joy_type == JOY_IDLE) {
    //     joy_status.seq = 3;//自动
    // } else {//JOY_ARM,JOY_SPEED
    //     joy_status.seq = 2;//手动
    // }
    // joy_status.frame_id = "joy_status";
    // pub_joy_status.publish(joy_status);
    // zs_add

    diagnostic_msgs::DiagnosticArray diag_array;
    diag_array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    if (topic_name == "/can0"){
        status.name = "estop/web";
    }else{
        status.name = "estop_web0";
    }
    if (stop_state) {
        status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
        status.message = "The emergency stop is pressed";
        diag_array.status.push_back(status);
        pub_diag.publish(diag_array);
        cmd.linear.x = 0;
        cmd.angular.z = 0;
        cmd.linear.z = 1;  
        pub_cmd_vel.publish(cmd);
        
        cmd_joy.header.stamp = ros::Time::now();
        cmd_joy.header.frame_id = "jtc_" + topic_name;
        cmd_joy.axes.resize(6);
        cmd_joy.axes[0] =  0;
        cmd_joy.axes[1] =  0;
        cmd_joy.axes[2] =  0;
        cmd_joy.axes[3] =  0;
        cmd_joy.axes[4] =  0;
        cmd_joy.axes[5] =  0;
        pub_arm_vel.publish(cmd_joy);  
    }else{
        status.level = diagnostic_msgs::DiagnosticStatus::OK;
        status.message = "Estop web checks out OK";
        diag_array.status.push_back(status);
        pub_diag.publish(diag_array);
    }


    if (ros::Time::now() - emergence_time < ros::Duration(1.0)) {  // 急停按下
        // trajectory_msgs::JointTrajectory trajectory;
        std_msgs::Bool trajectory;
        // trajectory.header.stamp = ros::Time::now();
        trajectory.data = true;
        // pub_arm_cancel_goal.publish(trajectory);
        cmd.linear.x = 0;
        cmd.angular.z = 0;
        cmd.linear.z = 1;  // 1急停按下;2手动;3自动

       
        pub_cmd_vel.publish(cmd);
        
        cmd_joy.header.stamp = ros::Time::now();
        cmd_joy.header.frame_id = "jtc_" + topic_name;
        cmd_joy.axes.resize(6);
        cmd_joy.axes[0] =  0;
        cmd_joy.axes[1] =  0;
        cmd_joy.axes[2] =  0;
        cmd_joy.axes[3] =  0;
        cmd_joy.axes[4] =  0;
        cmd_joy.axes[5] =  0;
        pub_arm_vel.publish(cmd_joy);  
    }
    switch (joy_type) {
        case JOY_SPEED:
            pub_cmd_vel.publish(cmd);
            last_joy_type = joy_type;
            break;
        case JOY_ARM:
            // to do
            cmd.linear.z = 2;  // 1急停按下;2手动;3自动
            cmd.angular.z = 0;
            cmd.linear.x = 0;
            pub_cmd_vel.publish(cmd);
            pub_arm_vel.publish(cmd_joy);
            last_joy_type = joy_type;
            break;
        case JOY_STOP:
            // to doh(cmd);
            cmd.linear.x = 0;
            cmd.angular.z = 0;
            cmd.linear.z = 1;  // 1急停按下;2手动;3自动
        
            pub_cmd_vel.publish(cmd);
            
            cmd_joy.header.stamp = ros::Time::now();
            cmd_joy.header.frame_id = "jtc_" + topic_name;
            cmd_joy.axes.resize(6);
            cmd_joy.axes[0] =  0;
            cmd_joy.axes[1] =  0;
            cmd_joy.axes[2] =  0;
            cmd_joy.axes[3] =  0;
            cmd_joy.axes[4] =  0;
            cmd_joy.axes[5] =  0;
            pub_arm_vel.publish(cmd_joy); 
            last_joy_type = joy_type;
            break;
        case JOY_IDLE:
            if (last_joy_type != JOY_IDLE) {
                cmd.linear.x = 0;
                cmd.angular.z = 0;
                pub_cmd_vel.publish(cmd);
                
                cmd_joy.header.stamp = ros::Time::now();
                cmd_joy.header.frame_id = "jtc_" + topic_name;
                cmd_joy.axes.resize(6);
                cmd_joy.axes[0] =  0;
                cmd_joy.axes[1] =  0;
                cmd_joy.axes[2] =  0;
                cmd_joy.axes[3] =  0;
                cmd_joy.axes[4] =  0;
                cmd_joy.axes[5] =  0;
                pub_arm_vel.publish(cmd_joy);  
            }
            last_joy_type = joy_type;
            break;
        default:
            break;
    }
}
}  // namespace joy_to_cmd