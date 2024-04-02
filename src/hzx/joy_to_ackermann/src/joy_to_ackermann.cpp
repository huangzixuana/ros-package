#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

double theta;
double prev_angular_z = 0;
double maxtheta;
double deltatheta;
std::string config_file_path = "/home/hzx/catkin_ws/src/hzx/joy_to_ackermann/config/ackermann.yaml";
ros::Publisher ackermann_pub;

void saveThetaToYaml(double theta, const std::string& file_path) {
  YAML::Node config;
  config["theta"] = theta;
  config["maxtheta"] = maxtheta;
  config["deltatheta"] = deltatheta;
  std::ofstream file(file_path);
  file << config;
}

// 回调函数：处理cmd_topic_name话题的消息
void cmdTopicCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  if (msg->header.frame_id != "/can0")
        return;
  if ((fabs(msg->axes[2]) == 0.5 && 
     fabs(msg->axes[5]) == 0 &&
     msg->buttons[0] == 1 &&
     msg->buttons[6] != 1))
    {  
      // 获取线速度和角速度
      double speed = msg->axes[1];
      double angular_z = msg->axes[3];

      // 创建ackermann消息
      ackermann_msgs::AckermannDrive ack_msg;
      ack_msg.speed = speed;

      // 根据规则计算steering_angle
      if (angular_z > 0)
      {
        if (angular_z >= prev_angular_z)
        {
          prev_angular_z = angular_z;
          if (theta < maxtheta)
          {
            theta += deltatheta;
          }
        }
      }
      else if (angular_z < 0)
      {
        if (angular_z <= prev_angular_z)
        {
          prev_angular_z = angular_z;
          if (theta > -maxtheta)
          {
            theta -= deltatheta;
          }
        }
      }
      else // angular_z == 0
      {
        prev_angular_z = 0;
      }

      ack_msg.steering_angle = theta;

      // 发布joy_to_ackermann话题
      ackermann_pub.publish(ack_msg);
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy_to_ackermann");

  ros::NodeHandle nh;
  ros::NodeHandle ph("~");

  YAML::Node config = YAML::LoadFile(config_file_path);
  if (config["theta"]) {
    theta = config["theta"].as<double>();
  } else {
    theta = 0.0;
  }
  maxtheta = config["maxtheta"].as<double>();
  deltatheta = config["deltatheta"].as<double>();

  ackermann_pub = nh.advertise<ackermann_msgs::AckermannDrive>("joy_to_ackermann", 1);

  ros::Subscriber cmd_sub = nh.subscribe("joy", 1, cmdTopicCallback);

  ros::Rate rate(10.0);
  int loop_count = 0;
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
    ++loop_count;
    if (loop_count == 600) { // 100 loops for 10 seconds
      saveThetaToYaml(theta, config_file_path); // Save data every 60 seconds
      loop_count = 0; // Reset loop count
    }
  }

  saveThetaToYaml(theta, config_file_path);

  return 0;
}