#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <ros/package.h>

double theta = 0;
double prev_angular_z = 0;
double maxtheta;
double deltatheta;
bool received_odom_msg = false;
ros::Publisher ackermann_pub;

// 读取yaml文件位置
std::string getConfigFilePath() {
    std::string package_path = ros::package::getPath("joy_to_ackermann");
    return package_path + "/config/ackermann.yaml";
}

// 读取参数值
void readConfigParameters(const std::string& config_file_path) {
    YAML::Node config = YAML::LoadFile(config_file_path);
    maxtheta = config["maxtheta"].as<double>();
    deltatheta = config["deltatheta"].as<double>();
}

// 回调函数：接收初始theta值
void odomCallback(const ackermann_msgs::AckermannDrive::ConstPtr& msg)
{
    // 只有在没有接收到过消息时才更新theta的值
    if (!received_odom_msg) {
        theta = msg->steering_angle;
        received_odom_msg = true;
    }
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
  std::string config_file_path = getConfigFilePath();
  readConfigParameters(config_file_path);

  ackermann_pub = nh.advertise<ackermann_msgs::AckermannDrive>("joy_to_ackermann", 1);
  ros::Subscriber odom_sub = nh.subscribe("ackermann_drive_odom", 1, odomCallback);
  ros::Subscriber cmd_sub = nh.subscribe("joy", 1, cmdTopicCallback);

  ros::Rate rate(10.0);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}