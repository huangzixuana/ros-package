#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <cmath>

class ImuMonitorNode {
public:
    ImuMonitorNode() {
        // 设置话题订阅者
        sub_imu_trig = nh.subscribe("/imu/trig", 10, &ImuMonitorNode::commandCallback, this);
        sub_imu_data = nh.subscribe("/imu/data", 10, &ImuMonitorNode::imuCallback, this);

        // 设置话题发布者
        pub_stop = nh.advertise<std_msgs::String>("plc24_request", 10);

        is_recording = false;
        last_exceeded_threshold = false;
        nh.param("imu_acceleration_monitor/threshold_value", threshold, std::double_t(0.5));
    }

    void commandCallback(const std_msgs::String::ConstPtr& msg) {
        if (msg->data == "start") {
            is_recording = true;
        } else if (msg->data == "stop") {
            is_recording = false;
            baseline_acceleration = {};
        }
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        if (is_recording) {
            if (baseline_acceleration.x == 0 && baseline_acceleration.y == 0 && baseline_acceleration.z == 0) {
                // 记录初始的加速度
                baseline_acceleration = msg->linear_acceleration;
            } else {
                // 比较当前加速度与基线加速度的差值
                double delta_x = fabs(msg->linear_acceleration.x - baseline_acceleration.x);
                double delta_y = fabs(msg->linear_acceleration.y - baseline_acceleration.y);
                double delta_z = fabs(msg->linear_acceleration.z - baseline_acceleration.z);

                bool current_exceeds_threshold = (delta_x > threshold || delta_y > threshold || delta_z > threshold);
                std_msgs::String stop_msg;

                if (current_exceeds_threshold && !last_exceeded_threshold) {
                    stop_msg.data = "estop_on";
                    pub_stop.publish(stop_msg);
                } else if (!current_exceeds_threshold && last_exceeded_threshold) {
                    stop_msg.data = "estop_off";
                    pub_stop.publish(stop_msg);
                    is_recording = false;
                }

                // 更新上一次是否超过阈值的状态
                last_exceeded_threshold = current_exceeds_threshold;
            }
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber sub_imu_trig;
    ros::Subscriber sub_imu_data;
    ros::Publisher pub_stop;

    bool is_recording;
    bool last_exceeded_threshold;
    geometry_msgs::Vector3 baseline_acceleration= {}; // 记录的基线加速度
    double threshold; // 阈值
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_acceleration_monitor"); // 初始化节点
    ImuMonitorNode node; // 创建节点对象

    ros::spin(); // 循环处理
    return 0;
}