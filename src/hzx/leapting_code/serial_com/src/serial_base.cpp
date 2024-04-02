/*
 * @Author: Hambin.Lu
 * @Description: ##
 */

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include "../include/serial_com/serial_base.hpp"

namespace serial_com
{

  SerialBase::~SerialBase()
  {
    CloseSerial();
  }

  bool SerialBase::OpenSerial(void)
  {
    serial::Timeout to = serial::Timeout::simpleTimeout(5000);
    _com.setPort(_com_name);
    _com.setBaudrate(_boundrate);
    _com.setTimeout(to);

    try
    {
      _com.open();
    }
    catch (serial::IOException &e)
    {
      ROS_ERROR(" %s is opened error,%s.", _com_name.c_str(), e.what());
      serial_diagarr.header.stamp = ros::Time::now();
      serial_diagarr.status[0].level = diagnostic_msgs::DiagnosticStatus::ERROR;
      serial_diagarr.status[0].message = "opened error!";
      serial_diag_pub.publish(serial_diagarr);
      return false;
    }

    if (_com.isOpen())
    {
      ROS_INFO(" %s is opened ok.", _com_name.c_str());
      serial_diagarr.header.stamp = ros::Time::now();
      serial_diagarr.status[0].level = diagnostic_msgs::DiagnosticStatus::OK;
      serial_diagarr.status[0].message = "ok";
      serial_diag_pub.publish(serial_diagarr);
      isRunning = true;
      return true;
    }
    else
    {
      ROS_ERROR(" %s is opened error.", _com_name.c_str());
      serial_diagarr.header.stamp = ros::Time::now();
      serial_diagarr.status[0].level = diagnostic_msgs::DiagnosticStatus::ERROR;
      serial_diagarr.status[0].message = "opened error!";
      serial_diag_pub.publish(serial_diagarr);
      return false;
    }
  }

  bool SerialBase::CloseSerial(void)
  {
    if (_com.isOpen())
    {
      _com.close();
    }
    return true;
  }

  void SerialBase::SendData(std::vector<uint8_t> &data)
  {
    // for(int i=0;i<data.size();i++){
    //   std::cout<<std::hex<<std::setw(2)<<std::setfill('0')<<int(data[i])<<" ";

    // }
    // std::cout<<std::endl;
    boost::unique_lock<boost::mutex> lockDeal(mutexWrite);
    size_t n = _com.write(data.data(), data.size());
  }

  void SerialBase::ReceivedData(void)
  {

    while (isRunning)
    {
      size_t n = _com.available();
      // std::cout<<n<<std::endl;
      if (n != 0)
      {
        uint8_t buffer[1000];
        try
        {
          n = _com.read(buffer, n);
        }
        catch (const std::exception &e)
        {
          std::cerr << e.what() << '\n';
        }

        {
          boost::unique_lock<boost::mutex> lockDeal(mutexRead);
          for (auto i = 0; i < n; i++)
          {
            // std::cout << std::hex << (buffer[i] & 0xff) << " ";
            receive_data.push_back(buffer[i]);
          }
        }
      }
      ros::Rate(200).sleep();
    }
  }

  void SerialBase::Run(void)
  {
    if (!OpenSerial())
      return;

    if (isRunning)
    {
      received_data_thread = std::make_shared<std::thread>(std::thread(&SerialBase::ReceivedData, this));
      if (received_data_thread->joinable())
        received_data_thread->detach();
      deal_data_thread = std::make_shared<std::thread>(std::thread(&SerialBase::DataDeal, this));
      if (deal_data_thread->joinable())
        deal_data_thread->detach();
      decoder_data_thread = std::make_shared<std::thread>(std::thread(&SerialBase::DecoderThread, this));
      if (decoder_data_thread->joinable())
        decoder_data_thread->detach();
    }
  }

  void SerialBase::DecoderThread(void)
  {
    while (isRunning)
    {
      std::vector<uint8_t> data;
      {
        boost::unique_lock<boost::mutex> lockDeal(mutexDeal);
        if (!decoder_data.empty())
        {
          data = decoder_data.front();
          decoder_data.pop();
        }
      }
      if (!data.empty())
        DecoderData(data);
      ros::Rate(200).sleep();
    }
  }

  uint16_t SerialBase::Crc16Check(const std::vector<uint8_t> &data, int index, int len)
  {
    static const uint16_t crc16Table[] =
        {
            0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
            0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
            0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
            0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
            0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
            0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
            0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
            0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
            0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
            0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
            0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
            0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
            0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
            0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
            0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
            0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
            0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
            0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
            0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
            0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
            0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
            0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
            0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
            0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
            0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
            0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
            0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
            0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
            0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
            0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
            0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
            0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040};
    uint8_t buf;
    uint16_t crc16 = 0xFFFF;
    for (int i = index; i < index + len; i++)
    {
      buf = data.at(i) ^ crc16;
      crc16 >>= 8;
      crc16 ^= crc16Table[buf];
    }
    uint16_t result = 0;
    result = ((crc16 & 0xff00) >> 8) | ((crc16 & 0x00ff) << 8);
    return result;
  }

  uint8_t SerialBase::CheckSum(std::vector<uint8_t> data, int index, int size)
  {
    uint8_t sum = 0;
    uint8_t check_sum = 0;
    for (int i = index; i < index + size; i++)
    {
      sum += data[i];
    }
    check_sum = sum & 0xff;
    return check_sum;
  }

  uint8_t SerialBase::CheckXOR(std::vector<uint8_t> &data, int index, int size)
  {
    uint8_t checkxor = 0;
    for (int i = index; i < size + index; i++)
    {
      checkxor = checkxor ^ data[i];
    }
    return checkxor;
  }


	uint8_t SerialBase::CheckLRC(std::vector<uint8_t> &data,int index, int size){
		uint8_t sum = 0;
		uint8_t check_sum = 0;
		for (int i = index; i < index+size; i++)
		{
			sum += data[i];
		}
		sum=~sum+1;
		check_sum = sum & 0xff;
		return check_sum;
	}

}
