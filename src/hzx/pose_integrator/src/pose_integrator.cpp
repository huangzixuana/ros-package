#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <deque>
#include <fstream>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

class PoseIntegrator
{
public:
    PoseIntegrator()
    {
        tf_yaml_file_ = ros::package::getPath("pose_integrator") + "/config/tf.yaml";
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
            return;  // 如果任一pose没有被初始化，就返回
        }

        // 如果协方差满足条件
        if (isCovarianceValid(pose_) && isCovarianceValid(pose_reflection_))
        {
            // 计算tf转化关系并存入队列中
            calculateAndStoreTransform(pose_, pose_reflection_, tf_buffer_);

            // 如果列表中存储了100个tf关系，就计算平均并发布
            if (tf_buffer_.size() == 100)
            {
                // 选择中间的transform
                selectMiddleTransforms();
                // 计算平均的transform
                computeAverageTransform();
                // 将平均的transform写入yaml文件
                writeTFRelationsToYAML();
                // 舍弃最早的数据
                tf_buffer_.pop_front();
            }
            else
            {
                readTFRelationsFromYAML();
            }
        }
        else
        {
            ROS_WARN("Covariance conditions not met, clearing TF buffer and restarting accumulation.");
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
    std::string tf_yaml_file_;

    bool isCovarianceValid(const geometry_msgs::PoseWithCovarianceStamped& pose)
    {
        // 检查协方差的第0，7，35位的和是否满足阈值
        double sum = pose.pose.covariance[0] + pose.pose.covariance[7] + pose.pose.covariance[35];
        // ROS_INFO("Sum of covariance elements: %f", sum);
        return sum < 0.02;
    }

    void calculateAndStoreTransform(const geometry_msgs::PoseWithCovarianceStamped& pose, 
                                    const geometry_msgs::PoseWithCovarianceStamped& pose_reflection, 
                                    std::deque<tf2::Transform>& tf_buffer) 
    {
        tf2::Transform tf_pose, tf_pose_reflection;
        tf2::fromMsg(pose.pose.pose, tf_pose);
        tf2::fromMsg(pose_reflection.pose.pose, tf_pose_reflection);

        // 计算从pose到pose_reflection的转换关系
        tf2::Transform transform = tf_pose.inverseTimes(tf_pose_reflection);

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

    void computeAverageTransform()
    {
        // Initialize the average transform
        average_transform.setIdentity();

        // Compute the sum of all transforms in the buffer
        for (const auto& tf : tf1_buffer_)
        {
            average_transform *= tf;
        }

        // Normalize the average transform
        double num_transforms = static_cast<double>(tf1_buffer_.size());
        average_transform.setOrigin(average_transform.getOrigin() / num_transforms);
        tf2::Quaternion avg_quaternion = average_transform.getRotation();
        avg_quaternion.normalize();
        average_transform.setRotation(avg_quaternion);
    }

    void publishAverageTransform(const tf2::Transform& transform)
    {
        // 发布最新的transform
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "map1";
        transformStamped.transform = tf2::toMsg(transform);
        broadcaster_.sendTransform(transformStamped);
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

    void readTFRelationsFromYAML()
    {
        // Open YAML file for reading
        std::ifstream yaml_file(tf_yaml_file_);
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