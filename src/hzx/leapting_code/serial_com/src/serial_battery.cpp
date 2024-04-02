#include "serial_base.hpp"
#include <ros/ros.h>
#include<sensor_msgs/BatteryState.h>
#include <bitset>
namespace serial_com
{


class SerialBattery : public SerialBase{
public:
    SerialBattery(ros::NodeHandle nh);
    void Timer1HzCallBack(const ros::TimerEvent &);
    ~SerialBattery();
    void DataDeal(void);
    void DecoderData(std::vector<uint8_t> &data);
    void SendData();

private:
    ros::Publisher _battery_status_pub;
    ros::Timer _send_data;
}; 

SerialBattery::SerialBattery(ros::NodeHandle nh){
    nhandle=nh;
	decoder_data_length=58;
	
    nh.getParam(ros::this_node::getName()+"/com_name", _com_name);
    nh.getParam(ros::this_node::getName()+"/boundrate", _boundrate);
	// _com_name="/dev/ttyUSB0";
	// _boundrate=9600;
	
    Run();
    
    if (!isRunning)
        //ros::shutdown();'
        exit(0);
    _battery_status_pub=nhandle.advertise<sensor_msgs::BatteryState>("/battery_status",3);
    _send_data=nhandle.createTimer(ros::Duration(1),&SerialBattery::Timer1HzCallBack,this);
}

SerialBattery::~SerialBattery(){
    isRunning = false;
}

void SerialBattery::SendData(){
  boost::unique_lock<boost::mutex> lockDeal(mutexWrite);
	std::vector<uint8_t> data={0x7F,0x10,0x02,0x1E,0x10,0x41};
  _com.write(data.data(),data.size());
}

void SerialBattery::Timer1HzCallBack(const ros::TimerEvent &){
  std::vector<uint8_t>  data{0x01, 0x03, 0x00, 0x70, 0x00, 0x00, 0x44, 0x11};
  SendData();
}

void SerialBattery::DataDeal(void){
    while (isRunning)
      {
        ros::Rate(200).sleep();
        boost::unique_lock<boost::mutex> lockDeal(mutexRead);
        int length = receive_data.size();
        if (length >= decoder_data_length)
        {
          for (int i = 0; i < length; i++)
          {
            if (i + decoder_data_length <= length)
            {
              if (
                  receive_data[i] == 0x7F && receive_data[i+1] == 0x10 && receive_data[i+2]==02 && receive_data[i+57]==CheckLRC(receive_data,i,decoder_data_length-1))
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


void SerialBattery::DecoderData(std::vector<uint8_t> &data){
	sensor_msgs::BatteryState battery_status;
	battery_status.header.stamp=ros::Time::now();
	battery_status.temperature=(data[47]+data[48]+data[49]+data[50])/4;
	battery_status.voltage=(data[5]*256+data[6])*0.01;
	battery_status.percentage=(data[7]*256+data[8])*0.01;
	battery_status.charge=data[9]*256+data[10];
	battery_status.current=(data[11]*256+data[12])*0.01;
	battery_status.cell_voltage.resize(16);
	battery_status.cell_voltage[0]=(data[14]*256+data[15])*0.001;
	battery_status.cell_voltage[1]=(data[16]*256+data[17])*0.001;
	battery_status.cell_voltage[2]=(data[18]*256+data[19])*0.001;
	battery_status.cell_voltage[3]=(data[20]*256+data[21])*0.001;
	battery_status.cell_voltage[4]=(data[22]*256+data[23])*0.001;
	battery_status.cell_voltage[5]=(data[24]*256+data[25])*0.001;
	battery_status.cell_voltage[6]=(data[26]*256+data[27])*0.001;
	battery_status.cell_voltage[7]=(data[28]*256+data[29])*0.001;
	battery_status.cell_voltage[8]=(data[30]*256+data[31])*0.001;
	battery_status.cell_voltage[9]=(data[32]*256+data[33])*0.001;
	battery_status.cell_voltage[10]=(data[34]*256+data[35])*0.001;
	battery_status.cell_voltage[11]=(data[36]*256+data[37])*0.001;
	battery_status.cell_voltage[12]=(data[38]*256+data[39])*0.001;
	battery_status.cell_voltage[13]=(data[40]*256+data[41])*0.001;
	battery_status.cell_voltage[14]=(data[42]*256+data[43])*0.001;
	battery_status.cell_voltage[15]=(data[44]*256+data[45])*0.001;
	battery_status.cell_temperature.resize(4);
	battery_status.cell_temperature[0]=data[47];
	battery_status.cell_temperature[1]=data[48];
	battery_status.cell_temperature[2]=data[49];
	battery_status.cell_temperature[3]=data[50];
	battery_status.power_supply_technology=4;
	std::bitset<8> bits1(data[53]);
	std::bitset<8> bits2(data[54]);
	std::bitset<8> bits3(data[55]);
	if(bits1[2]){
		battery_status.power_supply_health=4;
	}
	else if(bits1[7] || bits2[0]){
		battery_status.power_supply_health=2;
	}
	else if(bits1[8] || bits2[1]){
		battery_status.power_supply_health=6;
	}
	else if(!data[53] && !data[54] && !data[55] ){
		battery_status.power_supply_health=1;
	}
	else battery_status.power_supply_health=0;

	std::bitset<8> bits4(data[56]);
	if (bits4[0]){
	battery_status.power_supply_status=1;
	}
	else if(bits4[1]){
	battery_status.power_supply_status=2;
	}
	else battery_status.power_supply_status=0;
	battery_status.present=true;

	_battery_status_pub.publish(battery_status);
}
    


} // namespace serial_com

int main(int argc,char ** argv){

	ros::init(argc,argv,"battery_status_node");
	ros::NodeHandle nh;
	serial_com::SerialBattery battery_node(nh);
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::waitForShutdown();
	return 0;
}
