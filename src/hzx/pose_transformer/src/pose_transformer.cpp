#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

class PoseTransformer {
public:
    PoseTransformer() 
    {
        // 从参数服务器获取目标坐标系名称，默认值为 "map1"
        std::string node_name = ros::this_node::getName();
        nh_.param(node_name + "/target_frame", target_frame_, std::string());

        sub_ = nh_.subscribe("init_pose", 10, &PoseTransformer::poseCallback, this);
        pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("transformed_pose", 10);

        // 创建一个监听器来接收tf变换
        tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));
    }

    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        // 将接收到的姿态消息从map坐标系转换到map1坐标系
        geometry_msgs::PoseWithCovarianceStamped transformed_pose;
        try {
            geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform(target_frame_, msg->header.frame_id, ros::Time(0));
            tf2::doTransform(*msg, transformed_pose, transform_stamped);
        } catch (tf2::TransformException& ex) {
            ROS_WARN("Transform failed: %s", ex.what());
            return;
        }

        // 发布转换后的姿态消息
        pub_.publish(transformed_pose);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string target_frame_;

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_transformer_node");
    PoseTransformer transformer;
    ros::spin();
    return 0;
}