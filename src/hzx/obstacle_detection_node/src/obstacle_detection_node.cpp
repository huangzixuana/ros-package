#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher pub;
void pubtrig();
int numPoints1 = 0;
int numPoints2 = 0;

// 回调函数，用于处理接收到的点云数据
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{   
    
    // 将ROS消息转换为PCL点云类型
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pclCloud);

    // 获取激光点数量
    numPoints1 = pclCloud->size();
    ROS_INFO("the num of points1: %d", numPoints1);

    pubtrig();
}

void pointCloudCallback2(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // 将ROS消息转换为PCL点云类型
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pclCloud);

    // 获取激光点数量
    numPoints2 = pclCloud->size();
    ROS_INFO("the num of points2: %d", numPoints2);

    pubtrig();
}

void pubtrig() {
    std_msgs::Header trigMsg;
    trigMsg.stamp = ros::Time::now();
    trigMsg.frame_id = "obstacle_detected";
    if (numPoints1 + numPoints2 > 10) 
    {
        trigMsg.seq = 1;
    } 
    else 
    {
        trigMsg.seq = 0;
    }
    pub.publish(trigMsg);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_detection_node");
    ros::NodeHandle nh;

    pub = nh.advertise<std_msgs::Header>("/obstacle_trig", 10); // 创建发布 trig 话题的对象
    ros::Subscriber sub1 = nh.subscribe<sensor_msgs::PointCloud2>
        ("mid360_back_crop2", 10, pointCloudCallback); // 创建订阅 point_cloud 话题的对象
    ros::Subscriber sub2 = nh.subscribe<sensor_msgs::PointCloud2>
        ("mid360_front_crop2", 10, pointCloudCallback2); // 创建订阅 point_cloud 话题的对象
    ros::spin();
    return 0;
}