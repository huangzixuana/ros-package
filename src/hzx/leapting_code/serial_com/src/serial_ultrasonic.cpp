#include "serial_base.hpp"
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <bitset>
#include <sensor_msgs/LaserScan.h>
namespace serial_com
{

  class UltrasonicNode : public SerialBase
  {
  public:
    UltrasonicNode(ros::NodeHandle nh);
    void PublishData(void);
    ~UltrasonicNode();
    void DataDeal(void);
    void DecoderData(std::vector<uint8_t> &data);

  private:
    ros::Publisher _ultrasonic_pub;
    ros::Timer _send_data;
    float _DistanceArray[4]{0};
    float _distance;
    float _offset;

    sensor_msgs::LaserScan _scan_msg;
  };

  UltrasonicNode::UltrasonicNode(ros::NodeHandle nh)
  {
    nhandle = nh;
    decoder_data_length = 10;
    nh.getParam(ros::this_node::getName() + "/usb_name", _com_name);
    nh.getParam(ros::this_node::getName() + "/boundrate", _boundrate);
    nh.getParam(ros::this_node::getName() + "/distance", _distance);
    nh.getParam(ros::this_node::getName() + "/offset", _offset);

    Run();

    if (!isRunning)
      return;
    _ultrasonic_pub = nhandle.advertise<sensor_msgs::LaserScan>("/ultrasonic", 3);
    std::vector<uint8_t> data{0x01, 0x06, 0x02, 0x02, 0x00, 0x01, 0xE8, 0x72};
    SendData(data);
  }
  UltrasonicNode::~UltrasonicNode()
  {
    isRunning = false;
  }

  void UltrasonicNode::PublishData(void)
  {
    _scan_msg.header.stamp = ros::Time::now();
    _scan_msg.header.frame_id = "base_link";
    _scan_msg.angle_min = -0.52;

    _scan_msg.angle_max = 3.66;
    _scan_msg.angle_increment = 0.087;
    _scan_msg.scan_time = 0.14;
    _scan_msg.range_min = 0.008;
    _scan_msg.range_max = 3.5;
    int size = (_scan_msg.angle_max - _scan_msg.angle_min) / _scan_msg.angle_increment;

    _scan_msg.ranges.resize(size);
    for (int i = 0; i < size; i++)
      _scan_msg.ranges[i] = 20000.0;
    int points = (-_scan_msg.angle_min) / _scan_msg.angle_increment;
    // std::cout<<"_DistanceArray[0]:"<<_DistanceArray[0]<<"  _DistanceArray[1]:"<<_DistanceArray[1]
    //<<"  _DistanceArray[2]:"<<_DistanceArray[2]<<"  _DistanceArray[3]:"<<_DistanceArray[3]<<std::endl;
    for (int i = 0; i < points; i++)
    {
      _scan_msg.ranges[i] = (_DistanceArray[3] < _distance) ? _DistanceArray[3] : 20000;
      _scan_msg.ranges[i + points] = (_DistanceArray[2] < _distance) ? _DistanceArray[2] : 20000;
      _scan_msg.ranges[i + size - 2 * points] = (_DistanceArray[0] < _distance) ? _DistanceArray[0] : 20000;
      _scan_msg.ranges[i + size - points] = (_DistanceArray[1] < _distance) ? _DistanceArray[1] : 20000;
    }
    _ultrasonic_pub.publish(_scan_msg);
  }

  void UltrasonicNode::DataDeal(void)
  {
    while (isRunning)
    {
      ros::Rate(20).sleep();
      boost::unique_lock<boost::mutex> lockDeal(mutexRead);
      int length = receive_data.size();
      if (length >= decoder_data_length)
      {
        for (int i = 0; i < length; i++)
        {
          if (i + decoder_data_length <= length)
          {
            if (receive_data[i] == 0xFF && receive_data[i + decoder_data_length - 1] == CheckSum(receive_data, i, decoder_data_length - 1))
            {
              std::vector<uint8_t> cmd;
              for (int j = 0; j < decoder_data_length; j++)
                cmd.push_back(receive_data[i + j]);
              {
                boost::unique_lock<boost::mutex> lockDeal(mutexDeal);
                decoder_data.push(cmd);
              }

              receive_data.erase(receive_data.begin(), receive_data.begin() + decoder_data_length + i);
              break;
            }
          }
        }
      }
    }
  }

  void UltrasonicNode::DecoderData(std::vector<uint8_t> &data)
  {
    _DistanceArray[0] = (data[1] * 256 + data[2]) / 1000.0 + _offset;
    _DistanceArray[1] = (data[3] * 256 + data[4]) / 1000.0 + _offset;
    _DistanceArray[2] = (data[5] * 256 + data[6]) / 1000.0 + _offset;
    _DistanceArray[3] = (data[7] * 256 + data[8]) / 1000.0 + _offset;
    PublishData();
  }

} // namespace serial_com

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ultrasonic_node");
  ros::NodeHandle nh;
  serial_com::UltrasonicNode ultrasonic_node(nh);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
