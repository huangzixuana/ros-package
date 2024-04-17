#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <deque>
#include <fstream>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

class PoseCorrection
{
public:
    PoseCorrection()
    {
        tf_yaml_file_ = ros::package::getPath("pose_integrator") + "/config/tf.yaml";
        // 订阅amcl_pose和amcl_pose_reflection话题
        pose_sub_ = nh_.subscribe("amcl_pose", 10, &PoseCorrection::poseCallback, this);
        pose_reflection_sub_ = nh_.subscribe("amcl_pose_reflection", 10, &PoseCorrection::poseReflectionCallback, this);
         // 初始化发布者
        pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("selected_pose", 10);
    }

    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        pose_ = *msg;
        computeTransformAndPublish();
    }

    void poseReflectionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        pose_reflection_ = *msg;
        computeTransformAndPublish();
    }

    void computeTransformAndPublish()
    {
        if (pose_.header.stamp.isZero() || pose_reflection_.header.stamp.isZero())
        {
            // 如果任一pose没有被初始化，就返回
            return;
        }

        // 检查协方差是否满足条件
        if (!isCovarianceValid(pose_) || !isCovarianceValid(pose_reflection_))
        {
            ROS_WARN("Covariance conditions not met, clearing TF buffer and restarting accumulation.");
            // 清空列表
            tf_buffer_.clear();
            return;
        }

        // 计算pose之间的差异
        tf2::Transform tf_pose, tf_pose_reflection;
        tf2::fromMsg(pose_.pose.pose, tf_pose);
        tf2::fromMsg(pose_reflection_.pose.pose, tf_pose_reflection);

        // 计算从map到map1的转换关系
        tf2::Transform transform = tf_pose.inverseTimes(tf_pose_reflection);

        // 将计算的tf关系存储到列表中
        tf_buffer_.push_back(transform);

        // 如果列表中存储了100个tf关系，就计算平均并发布
        if (tf_buffer_.size() == 100)
        {
            computeAverageTransform();
            // 舍弃最早的数据
            tf_buffer_.pop_front();
        }
        // 发布平均的transform
        publishLatestTransform(average_transform);

        // Write TF relations to YAML file
        writeTFRelationsToYAML();

        geometry_msgs::PoseWithCovarianceStamped selected_pose;
        if (pose_.pose.covariance[0] + pose_.pose.covariance[7] + pose_.pose.covariance[35] <
            pose_reflection_.pose.covariance[0] + pose_reflection_.pose.covariance[7] + pose_reflection_.pose.covariance[35])
        {
            selected_pose = pose_;
        }
        else
        {
            selected_pose = pose_reflection_;
        }

        // 发布选择的姿态
        pose_pub_.publish(selected_pose);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::Subscriber pose_reflection_sub_;
    ros::Publisher pose_pub_; 
    geometry_msgs::PoseWithCovarianceStamped pose_, pose_reflection_;
    std::deque<tf2::Transform> tf_buffer_;
    tf2_ros::TransformBroadcaster broadcaster_;
    tf2::Transform average_transform;
    std::string tf_yaml_file_;

    bool isCovarianceValid(const geometry_msgs::PoseWithCovarianceStamped& pose)
    {
        // 检查协方差的第0，7，35位的和是否小于0.5
        double sum = pose.pose.covariance[0] + pose.pose.covariance[7] + pose.pose.covariance[35];
        return sum < 0.5;
    }

    void publishLatestTransform(const tf2::Transform& transform)
    {
        // 发布最新的transform
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "map1";
        transformStamped.transform = tf2::toMsg(transform);
        broadcaster_.sendTransform(transformStamped);
    }

    void computeAverageTransform()
    {
        // Initialize the average transform
        average_transform.setIdentity();

        // Compute the sum of all transforms in the buffer
        for (const auto& tf : tf_buffer_)
        {
            average_transform *= tf;
        }

        // Normalize the average transform
        double num_transforms = static_cast<double>(tf_buffer_.size());
        average_transform.setOrigin(average_transform.getOrigin() / num_transforms);
        tf2::Quaternion avg_quaternion = average_transform.getRotation();
        avg_quaternion.normalize();
        average_transform.setRotation(avg_quaternion);
    }

    void writeTFRelationsToYAML()
    {
        // Open YAML file for writing
        std::ofstream yaml_file(tf_yaml_file_);

        // Write TF relations to YAML
        yaml_file << "tf_relations:\n";
        yaml_file << "  - average_transform:\n";
        yaml_file << "      translation:\n";
        yaml_file << "        x: " << average_transform.getOrigin().x() << "\n";
        yaml_file << "        y: " << average_transform.getOrigin().y() << "\n";
        yaml_file << "        z: " << average_transform.getOrigin().z() << "\n";
        yaml_file << "      rotation:\n";
        yaml_file << "        x: " << average_transform.getRotation().x() << "\n";
        yaml_file << "        y: " << average_transform.getRotation().y() << "\n";
        yaml_file << "        z: " << average_transform.getRotation().z() << "\n";
        yaml_file << "        w: " << average_transform.getRotation().w() << "\n";

        yaml_file.close();
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_correction");
    PoseCorrection pose_correction;
    ros::spin();
    return 0;
}