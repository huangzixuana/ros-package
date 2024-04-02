/*
 * @Author: Hambin.Lu
 * @Description: ##
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef SERIAL_BASE_HPP_
#define SERIAL_BASE_HPP_

#include <iostream>
#include <string>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <boost/thread/pthread/shared_mutex.hpp>
#include <ros/ros.h>
#include <serial/serial.h>
#include <queue>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>

namespace serial_com
{

  class SerialBase
  {

  public:
    SerialBase()
    {
      diagnostic_msgs::DiagnosticStatus serial_diag;
      serial_diag.hardware_id = "none";
      serial_diag.name = "serial_usb";
      serial_diag.message = "OK";
      serial_diagarr.status.push_back(serial_diag);
      serial_diag_pub = nhandle.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics_agg", 1, true);
    }
    virtual ~SerialBase();

    ros::NodeHandle nhandle;
    ros::Publisher serial_diag_pub;
    boost::mutex mutexRead;
    boost::mutex mutexWrite;
    boost::mutex mutexDeal;
    

  protected:
    bool isRunning = false;
    serial::Serial _com;
    std::string _com_name;
    int _boundrate;

    std::vector<uint8_t> receive_data;
    std::queue<std::vector<uint8_t>> decoder_data;
    int decoder_data_length;

    void Run(void);
    uint16_t Crc16Check(const std::vector<uint8_t> &data, int index, int len);
    uint8_t CheckSum(std::vector<uint8_t> data, int index, int size);
    uint8_t CheckXOR(std::vector<uint8_t> &data, int index, int size);
    uint8_t CheckLRC(std::vector<uint8_t> &data,int index, int size);
    bool OpenSerial(void);
    bool CloseSerial(void);
    void SendData(std::vector<uint8_t> &data);
    void ReceivedData(void);
    virtual void DataDeal(void) = 0;
    virtual void DecoderData(std::vector<uint8_t> &data) = 0;
    void DecoderThread(void);

    template <typename T>
    T Uint8ToConvert(std::vector<uint8_t> &byte, int &count, T &invert)
    {
      if (typeid(invert).name() == typeid(int32_t).name() || typeid(invert).name() == typeid(uint32_t).name())
      {
        // invert = ((((((invert&0x00000000)|byte[count++]<<8)|byte[count++])<<8)|byte[count++])<<8)|byte[count++];
        invert = ((((invert & 0x00000000) | byte[count++]) | (byte[count++]) << 8) | (byte[count++]) << 16) | (byte[count++] << 24);
        return invert;
      }
      else if (typeid(invert).name() == typeid(int16_t).name() || typeid(invert).name() == typeid(uint16_t).name())
      {
        invert = ((invert & 0x0000) | byte[count++]) | (byte[count++] << 8);
        return invert;
      }
      else
        std::cout << "No input type!";
      return 0;
    }

    template <typename T>
    void ConvertToUint8(T iValue, std::vector<uint8_t> &bytes)
    {
      if (typeid(iValue).name() == typeid(int32_t).name() || typeid(iValue).name() == typeid(uint32_t).name())
      {
        bytes.push_back((uint8_t)((iValue & 0xFF000000) >> 24));
        bytes.push_back((uint8_t)((iValue & 0x00FF0000) >> 16));
        bytes.push_back((uint8_t)((iValue & 0x0000FF00) >> 8));
        bytes.push_back((uint8_t)(iValue & 0x000000FF));
      }
      else if (typeid(iValue).name() == typeid(int16_t).name() || typeid(iValue).name() == typeid(uint16_t).name())
      {
        bytes.push_back((uint8_t)((iValue & 0xFF00) >> 8));
        bytes.push_back((uint8_t)(iValue & 0x00FF));
      }
      else
        std::cout << "No input type!";
    }

  private:
    std::shared_ptr<std::thread> received_data_thread;
    std::shared_ptr<std::thread> deal_data_thread;
    std::shared_ptr<std::thread> decoder_data_thread;
    diagnostic_msgs::DiagnosticArray serial_diagarr;
    diagnostic_updater::DiagnosticStatusWrapper serial_diagwrap;
    
  };

}
#endif
