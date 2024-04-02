#include "serial_base.hpp"
#include <ros/ros.h>
#include<std_msgs/Int32MultiArray.h>
#include <bitset>
namespace serial_com
{


class RelayNode : public SerialBase{
public:
    RelayNode(ros::NodeHandle nh);
    void SubCallBack(const std_msgs::Int32MultiArray msg);
    void Timer10HzCallBack(const ros::TimerEvent &);
    ~RelayNode();
    void DataDeal(void);
    void DecoderData(std::vector<uint8_t> &data);

private:
    ros::Publisher _relay_status_pub;
    ros::Subscriber _relay_sub;
    ros::Timer _send_data;
	std_msgs::Int32MultiArray _msg;


}; 

RelayNode::RelayNode(ros::NodeHandle nh){
    nhandle=nh;
	decoder_data_length=6;
    nh.getParam(ros::this_node::getName()+"/usb_name", _com_name);
    nh.getParam(ros::this_node::getName()+"/boundrate", _boundrate);
	_msg.layout.dim.resize(2);
	_msg.layout.dim[0].size = 2; // 第一维度大小为2
	_msg.layout.dim[0].stride = 4; // 第一维度的步长为4（每行有4个元素）
  _msg.layout.dim[0].label="output";
	_msg.layout.dim[1].size = 4; // 第二维度大小为4
	_msg.layout.dim[1].stride = 1; // 第二维度的步长为1
  _msg.layout.dim[1].label="input";
	_msg.data.resize(8);
	
    Run();
    
    if (!isRunning)
        return;
    _relay_status_pub=nhandle.advertise<std_msgs::Int32MultiArray>("/relay_status",3);
    _relay_sub=nhandle.subscribe("/relay_control",3,&RelayNode::SubCallBack,this);
    _send_data=nhandle.createTimer(ros::Duration(0.1),&RelayNode::Timer10HzCallBack,this);

}
RelayNode::~RelayNode(){
    isRunning = false;

}
//1代表开，0代表关
void RelayNode::SubCallBack(const std_msgs::Int32MultiArray msg){
    
    for(int i=0;i<4;i++){
        std::vector<uint8_t>  data{0xFE, 0x05, 0x00};
        data.push_back(uint8_t(i));
        if(msg.data[i]){
            data.push_back(uint8_t(0xFF));
            data.push_back(uint8_t(0x00));
            uint16_t result=Crc16Check(data,0,6);
            data.push_back(uint8_t((result >> 8) & 0xFF));
            data.push_back(uint8_t(result & 0xFF));
			// for(const auto & da:data){
			// 	std::cout<<std::hex<<std::setw(2)<<std::setfill('0')<<static_cast<int>(da)<<" ";
			// }
			// std::cout<<std::endl;
            
        }
        else{	
            
            data.push_back(uint8_t(0x00));
            data.push_back(uint8_t(0x00));
            uint16_t result=Crc16Check(data,0,6);
            data.push_back(uint8_t((result >> 8) & 0xFF));
            data.push_back(uint8_t(result & 0xFF));
        }

		
		SendData(data);
		ros::Rate(50).sleep();
    }
    
}

void RelayNode::Timer10HzCallBack(const ros::TimerEvent &){
	
  std::vector<uint8_t>  data1{0xFE, 0x01, 0x00, 0x00, 0x00, 0x04, 0x29, 0xC6};
  std::vector<uint8_t>  data2{0xFE, 0x02, 0x00, 0x00, 0x00, 0x04, 0x6D, 0xC6};
  SendData(data1);
  usleep(20000);
  SendData(data2);
  _relay_status_pub.publish(_msg);
}

void RelayNode::DataDeal(void){
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
                  receive_data[i] == 0xFE )
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


void RelayNode::DecoderData(std::vector<uint8_t> &data){
	// for(int i=0;i<data.size();i++){

	// 	std::cout<<std::hex<<std::setw(2)<<std::setfill('0')<<int(data[i])<<" ";

	// }
	//std::cout<<std::endl;
	if(data[1]==0x01){
		std::bitset<4> bits(data[3]);
		
		
		_msg.data[0]=bits[0];
		_msg.data[1]=bits[1];
		_msg.data[2]=bits[2];
		_msg.data[3]=bits[3];
		
		
	}
	else {
    	// for(int i=0;i<data.size();i++){
      	// 	std::cout<<std::hex<<std::setw(2)<<std::setfill('0')<<int(data[i])<<" ";
    	// }
    	// std::cout<<std::endl;
		std::bitset<4> bits(data[3]);

		_msg.data[4]=bits[0];
		_msg.data[5]=bits[1];
		_msg.data[6]=bits[2];
		_msg.data[7]=bits[3];
		
	}
	
}
    


} // namespace serial_com

int main(int argc,char ** argv){

	ros::init(argc,argv,"relay_node");
	ros::NodeHandle nh;
	serial_com::RelayNode relay_node(nh);
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::waitForShutdown();
	return 0;
}
