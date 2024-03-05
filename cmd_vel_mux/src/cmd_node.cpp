/*
 * @Author: hambin
 * @Date: 2023-03-10 13:40:12
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or
 * install git & please set dead value or install git
 * @FilePath: src/cmd_vel_mux.cpp
 * @Description: can to joy message
 */
#include "cmd_node.h"
#include <signal.h>
namespace cmd_node {
CmdNode::CmdNode(ros::NodeHandle& nhandle,
                 std::string topic_name,
                 int pri,
                 float time,
                 std::string short_desc,
                 std::shared_ptr<int> g_pri,                // 全局优先级
                 std::shared_ptr<float> g_timeout,          // 全局持续时间
                 std::shared_ptr<ros::Time> g_change_time,  // 全局修改时间
                 std::string out_topic)
    : nh(nhandle),
      name(topic_name),
      priority(pri),
      timeout(time),
      global_priority(g_pri),
      global_timeout(g_timeout),
      global_change_time(g_change_time),
      cmd_topic_name(out_topic) {
    // 订阅输入速度
    sub_cmd = nh.subscribe(topic_name, 1, &CmdNode::CmdHandler, this);
    sub_slow_trig = nh.subscribe("/slow_trig", 1, &CmdNode::SlowTrigHandler, this);  // zs add
    // 发布输出速度
    pub_cmd_vel = nh.advertise<geometry_msgs::Twist>(cmd_topic_name, 1);
    pub_joy_status = nh.advertise<std_msgs::Header>("/joy_status", 1);  // zs add
}
void CmdNode::CmdHandler(const geometry_msgs::TwistConstPtr msg) {
    // 修改优先级
    if (priority <= *global_priority) {
        *global_priority = priority;
        *global_timeout = timeout;
        *global_change_time = ros::Time::now();
        // 发布命令
        // pub_cmd_vel.publish(*msg);
        // std::cout<<"pub_cmd: "<<name<<"  g_pri: "<<*global_priority<<" pri: "<<priority<<std::endl;
        // zs add
        geometry_msgs::Twist get_twist = *msg;
        get_twist.linear.z = 0.0;  // use to mode
        if (ros::Time::now() - slow_trig_time < ros::Duration(1.0)) {
            get_twist.linear.x = 0.5 * get_twist.linear.x;
            get_twist.angular.z = 0.5 * get_twist.angular.z;
        }
        pub_cmd_vel.publish(get_twist);
        // zs_add
        int mode = static_cast<int>(msg->linear.z);  // 1急停按下;2手动;3自动
        std_msgs::Header joy_status;
        if (mode == 1) {
            joy_status.seq = 1;  // 急停按下
        } else if (mode == 2) {
            joy_status.seq = 2;  // 手动
        } else if (mode == 3) {
            joy_status.seq = 3;  // 自动
        } else {                 // JOY_ARM,JOY_SPEED
            joy_status.seq = 3;  // 自动
        }
        joy_status.frame_id = "joy_status";
        pub_joy_status.publish(joy_status);
    }
}
void CmdNode::SlowTrigHandler(const std_msgs::HeaderConstPtr msg) {  // zs add
    if (msg->seq == 1) {
        slow_trig_time = ros::Time::now();
    }
}
}  // namespace cmd_node
