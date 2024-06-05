#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <deque>
#include <fstream>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <dynamic_reconfigure/server.h>
#include "pose_integrator/pose_integratorConfig.h"

class PoseIntegrator
{
public:
    PoseIntegrator()
    {   
        f = boost::bind(&PoseIntegrator::dynamicConfigCallback, this, _1, _2);
        server.setCallback(f);
        std::string yaml_read, yaml_write;
        std::string node_name = ros::this_node::getName();
        nh_.param("pose_integrator_node/yaml_read", yaml_read, std::string("/config/tf.yaml"));
        nh_.param("pose_integrator_node/yaml_write", yaml_write, std::string("/config/tf_write.yaml"));
        tf_yaml_file_read = ros::package::getPath("pose_integrator") + yaml_read;
        tf_yaml_file_write = ros::package::getPath("pose_integrator") + yaml_write;
        // 订阅amcl_pose和amcl_pose_reflection话题
        pose_sub_ = nh_.subscribe("amcl_pose", 10, &PoseIntegrator::poseCallback, this);
        pose_reflection_sub_ = nh_.subscribe("amcl_pose_reflection", 10, &PoseIntegrator::poseReflectionCallback, this);
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
            tf_buffer_.clear();
            return;  // 如果任一pose没有被初始化，就返回
        }

        readTFRelationsFromYAML();

        std::int32_t compute_average;
        nh_.param("pose_integrator_node/compute_average", compute_average, std::int32_t());

        // ROS_INFO("compute_average: %d", compute_average);

        std::int32_t tf_write;
        nh_.param("pose_integrator_node/tf_write", tf_write, std::int32_t(1));
        // ROS_INFO("tf_write: %d", tf_write);

        // 如果协方差满足条件
        if (isCovarianceValid(pose_) && isCovarianceValid(pose_reflection_) && compute_average == 1)
        {
            // 计算tf转化关系并存入队列中
            calculateAndStoreTransform(pose_, pose_reflection_, tf_buffer_);

            std::string node_name = ros::this_node::getName();
            std::int32_t tf_write;
            // nh_.param(node_name + "/queue_length", queue_length, std::int32_t(100));
            nh_.param("pose_integrator_node/tf_write", tf_write, std::int32_t(1));

            // 如果列表中存储了queue_length个tf关系，就计算平均并发布
            std::int32_t size = tf_buffer_.size();
            ROS_INFO("tf_buffer_size: %d", size);
            if (size >= queue_length)
            {
                // 选择中间的transform
                selectMiddleTransforms();
                // 计算平均的transform
                computeAverageTransform(tf1_buffer_);
                // 将平均的transform写入yaml文件
                if (tf_write == 1)
                {
                    writeTFRelationsToYAML();
                }
                // 舍弃最早的数据
                tf_buffer_.pop_front();
            }
        }
        else
        {
            // ROS_WARN("Covariance conditions not met, clearing TF buffer and restarting accumulation.");
            // 清空队列
            tf_buffer_.clear();
        }

        // 发布平均的transform
        publishAverageTransform(average_transform);  
        
        //发布pose
        publishSelectedPose(pose_, pose_reflection_, pose_pub_);

    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::Subscriber pose_reflection_sub_;
    ros::Publisher pose_pub_; 
    geometry_msgs::PoseWithCovarianceStamped pose_, pose_reflection_;
    std::deque<tf2::Transform> tf_buffer_;
    std::deque<tf2::Transform> tf1_buffer_;
    tf2_ros::TransformBroadcaster broadcaster_;
    tf2::Transform average_transform;
    std::string tf_yaml_file_read;
    std::string tf_yaml_file_write;
    std::string node_name;
    std::double_t covariance_threshold;
    std::int32_t queue_length;
    dynamic_reconfigure::Server<pose_integrator_cfg::pose_integratorConfig> server;
    dynamic_reconfigure::Server<pose_integrator_cfg::pose_integratorConfig>::CallbackType f;

    void dynamicConfigCallback(pose_integrator_cfg::pose_integratorConfig &config, uint32_t level)
    {
        covariance_threshold = config.Covariance_threshold;
        queue_length = config.Queue_length;
        // ROS_INFO("Covariance threshold value: %f", covariance_threshold);
        // ROS_INFO("Queue length value: %d", queue_length);
    }

    bool isCovarianceValid(const geometry_msgs::PoseWithCovarianceStamped& pose)
    {
        // std::double_t Covariance_threshold;
        // nh_.param("Covariance_threshold", Covariance_threshold, std::double_t());
        // 检查协方差的第0，7，35位的和是否满足阈值
        double sum = pose.pose.covariance[0] + pose.pose.covariance[7] + pose.pose.covariance[35];
        // ROS_INFO("Sum of covariance elements: %f", sum);
        // return sum < covariance_threshold;
        return sum < 0;
    }

    void calculateAndStoreTransform(const geometry_msgs::PoseWithCovarianceStamped& pose, 
                                    const geometry_msgs::PoseWithCovarianceStamped& pose_reflection, 
                                    std::deque<tf2::Transform>& tf_buffer) 
    {
        tf2::Transform tf_pose, tf_pose_reflection, transform;
        tf2::fromMsg(pose.pose.pose, tf_pose);
        tf2::fromMsg(pose_reflection.pose.pose, tf_pose_reflection);
        geometry_msgs::TransformStamped map_to_map1;
        geometry_msgs::TransformStamped trans;

        tf2::Transform inverse_tf_pose_reflection = tf_pose_reflection.inverse();
        geometry_msgs::TransformStamped tf_pose_reflection_inv;
        tf_pose_reflection_inv.header = pose_reflection.header;
        tf_pose_reflection_inv.transform = tf2::toMsg(inverse_tf_pose_reflection);

        trans.header = pose.header;
        trans.transform = tf2::toMsg(tf_pose);

        tf2::doTransform(tf_pose_reflection_inv,map_to_map1,trans);

        tf2::fromMsg(map_to_map1.transform, transform);

        // 计算从 map 到 map1 的变换
        // tf2::Transform transform = transform_pose_reflection.inverseTimes(transform_pose);

        geometry_msgs::TransformStamped transformStamped1;
        transformStamped1.header.stamp = ros::Time::now();
        transformStamped1.header.frame_id = "test";
        transformStamped1.child_frame_id = "test1";
        transformStamped1.transform = tf2::toMsg(transform);
        broadcaster_.sendTransform(transformStamped1);

        // 将计算的tf关系存储到队列中
        tf_buffer.push_back(transform);
    }

    void selectMiddleTransforms() 
    {
        // Store the sum of translation components for all transforms
        std::deque<std::pair<tf2::Vector3, size_t>> translation_sums;

        // Iterate through all transforms and calculate the sum of translation components
        for (size_t i = 0; i < tf_buffer_.size(); i++) 
        {
            tf2::Vector3 translation = tf_buffer_[i].getOrigin();
            translation_sums.push_back({translation, i});
        }

        // Sort based on the sum of absolute values of translation components
        std::sort(translation_sums.begin(), translation_sums.end(), [](const auto &lhs, const auto &rhs) 
        {
            auto sum_abs_lhs = std::abs(lhs.first.x()) + std::abs(lhs.first.y()) + std::abs(lhs.first.z());
            auto sum_abs_rhs = std::abs(rhs.first.x()) + std::abs(rhs.first.y()) + std::abs(rhs.first.z());
            return sum_abs_lhs < sum_abs_rhs;
        });

        // Find the range of indices for the middle 20 sums
        size_t start_index = (translation_sums.size() - 20) / 2;
        size_t end_index = start_index + 20;

        // Clear tf1_buffer_ and fill it with the middle 20 transforms
        tf1_buffer_.clear();
        for (size_t i = start_index; i < end_index; ++i) 
        {
            size_t transform_index = translation_sums[i].second;
            tf1_buffer_.push_back(tf_buffer_[transform_index]);
        }
    }

    tf2::Quaternion slerp(const tf2::Quaternion& q1, const tf2::Quaternion& q2, double t) {
    // 对两个四元数进行球面线性插值
        return q1.slerp(q2, t);
    }

    void computeAverageTransform(const std::deque<tf2::Transform>& transforms) 
    {
    // 分别存储所有 transform 的旋转和平移部分
        std::deque<tf2::Quaternion> rotations;
        std::deque<tf2::Vector3> translations;

        // 提取所有 transform 的旋转和平移部分
        for (const auto& transform : transforms) {
            rotations.push_back(transform.getRotation());
            translations.push_back(transform.getOrigin());
        }

        // 计算平均旋转（使用球面线性插值）
        tf2::Quaternion averageRotation = tf2::Quaternion::getIdentity();
        for (size_t i = 0; i < rotations.size(); ++i) {
            // 对每对相邻的旋转进行插值
            averageRotation = tf2::slerp(averageRotation, rotations[i], 1.0 / (i + 1));
        }

        // 计算平均平移
        tf2::Vector3 averageTranslation = tf2::Vector3(0, 0, 0);
        for (const auto& translation : translations) {
            averageTranslation += translation;
        }
        averageTranslation /= translations.size();

        average_transform.setRotation(averageRotation);
        average_transform.setOrigin(averageTranslation);
    }

    void publishAverageTransform(const tf2::Transform& transform)
    {
        // 发布最新的transform
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        std::string map, map1;
        std::string node_name = ros::this_node::getName();
        nh_.param("pose_integrator_node/init_frame", map, std::string("map"));
        nh_.param("pose_integrator_node/transformed_frame", map1, std::string("map1"));
        transformStamped.header.frame_id = map;
        transformStamped.child_frame_id = map1;
        transformStamped.transform = tf2::toMsg(transform);

        std::int32_t tf_publish;
        nh_.param("pose_integrator_node/tf_publish", tf_publish, std::int32_t(1));
        if (tf_publish == 1)
        {
            broadcaster_.sendTransform(transformStamped);
        }
    }

    void writeTFRelationsToYAML()
    {
        // Open YAML file for writing
        std::ofstream yaml_file(tf_yaml_file_write);

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

    void readTFRelationsFromYAML()
    {
        // Open YAML file for reading
        std::ifstream yaml_file(tf_yaml_file_read);
        // Check if the file is open
        if (!yaml_file.is_open())
        {
            std::cerr << "Error opening YAML file." << std::endl;
            return;
        }
        YAML::Node yaml_node = YAML::Load(yaml_file);
        // Check if the tf_relations key exists
        if (!yaml_node["tf_relations"])
        {
            std::cerr << "tf_relations key not found in YAML file." << std::endl;
            yaml_file.close();
            return;
        }
        YAML::Node average_transform_node = yaml_node["tf_relations"][0]["average_transform"];
        // Populate average_transform
        average_transform.setOrigin(tf2::Vector3(
            average_transform_node["translation"]["x"].as<double>(),
            average_transform_node["translation"]["y"].as<double>(),
            average_transform_node["translation"]["z"].as<double>()
        ));
        average_transform.setRotation(tf2::Quaternion(
            average_transform_node["rotation"]["x"].as<double>(),
            average_transform_node["rotation"]["y"].as<double>(),
            average_transform_node["rotation"]["z"].as<double>(),
            average_transform_node["rotation"]["w"].as<double>()
        ));
        // Close the file
        yaml_file.close();
    }

    void publishSelectedPose(const geometry_msgs::PoseWithCovarianceStamped& pose, const geometry_msgs::PoseWithCovarianceStamped& pose_reflection, ros::Publisher& pub) 
    {
        geometry_msgs::PoseWithCovarianceStamped selected_pose;

        if (pose.pose.covariance[0] + pose.pose.covariance[7] + pose.pose.covariance[35] <
            pose_reflection.pose.covariance[0] + pose_reflection.pose.covariance[7] + pose_reflection.pose.covariance[35])
            {
                selected_pose = pose;
            }
        else 
            {
                selected_pose = pose_reflection;
            }

        // 发布选择的姿态
        pub.publish(selected_pose);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_integrator");
    PoseIntegrator pose_integrator;
    ros::spin();
    return 0;
}