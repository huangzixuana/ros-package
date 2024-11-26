#include "include/CustomMsg.h"
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <chrono>
#include <thread> 

using namespace std;

string bag;
string lidar_topic;
string pcd;
string bag_file, pcd_file;
int data_num;
bool is_custom_msg;

void processBagFile(const string& bag_file, const string& pcd_file, bool is_custom_msg) {
    pcl::PointCloud<pcl::PointXYZI> output_cloud;

    rosbag::Bag bag;
    try {
        bag.open(bag_file, rosbag::bagmode::Read);
    } catch (const rosbag::BagException &e) {
        ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
        return;
    }

    vector<string> lidar_topic_vec = { lidar_topic };
    rosbag::View view(bag, rosbag::TopicQuery(lidar_topic_vec));
    
    for (const rosbag::MessageInstance &m : view) {
        if (is_custom_msg) {
            auto livox_cloud_msg = m.instantiate<livox_ros_driver::CustomMsg>();
            if (livox_cloud_msg) {
                for (uint i = 0; i < livox_cloud_msg->point_num; ++i) {
                    pcl::PointXYZI p;
                    p.x = livox_cloud_msg->points[i].x;
                    p.y = livox_cloud_msg->points[i].y;
                    p.z = livox_cloud_msg->points[i].z;
                    p.intensity = livox_cloud_msg->points[i].reflectivity;
                    output_cloud.points.push_back(p);
                }
            }
        } else {
            auto livox_cloud = m.instantiate<sensor_msgs::PointCloud2>();
            if (livox_cloud) {
                pcl::PointCloud<pcl::PointXYZI> cloud;
                pcl::PCLPointCloud2 pcl_pc;
                pcl_conversions::toPCL(*livox_cloud, pcl_pc);
                pcl::fromPCLPointCloud2(pcl_pc, cloud);
                output_cloud += cloud;
            }
        }
    }
    output_cloud.is_dense = false;
    output_cloud.width = output_cloud.points.size();
    output_cloud.height = 1;
    pcl::io::savePCDFileASCII(pcd_file, output_cloud);
    ROS_INFO_STREAM("Successfully saved point cloud to PCD file: " << pcd_file);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidarCamCalib");
    ros::NodeHandle nh;
    nh.param<string>("bag_file", bag, "");
    nh.param<string>("pcd_file", pcd, "");
    nh.param<string>("lidar_topic", lidar_topic, "/livox/lidar");
    nh.param<int>("data_num", data_num, 5);
    nh.param<bool>("is_custom_msg", is_custom_msg, false);

    for (int i = 0; i < data_num; ++i) {
        bag_file = bag + std::to_string(i) + ".bag";
        pcd_file = pcd + std::to_string(i) + ".pcd";

        if (!ros::ok()) break; // Check if ROS is still running

        processBagFile(bag_file, pcd_file, is_custom_msg);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}