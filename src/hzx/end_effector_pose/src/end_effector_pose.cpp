#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Header.h>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <thread>  
#include <chrono>  

moveit::core::RobotStatePtr kinematic_state;
bool Initialization = true;
ros::Publisher end_effector_pose_pub;

void PoseCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
    if (msg->points.empty()) {
        ROS_WARN("Received empty trajectory points.");
        return;
    }

    const int num_points = msg->points.size();
    bool all_within_range = true;
    ROS_INFO("num points: %d",num_points);
    for (int i = 0; i < num_points; ++i) 
    {
        const auto& joint_positions = msg->points[i].positions;
        std::vector<double> gstate(joint_positions.begin(), joint_positions.end());
        kinematic_state->setJointGroupPositions("leapting_arm", gstate);
        const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("link_6");
        Eigen::Matrix3d rotation_matrix = end_effector_state.rotation();
        Eigen::Quaterniond eigen_quaternion(rotation_matrix);
        tf::Quaternion tf_quaternion(eigen_quaternion.x(), eigen_quaternion.y(), eigen_quaternion.z(), eigen_quaternion.w());
        double roll, pitch, yaw;
        tf::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);
        // Eigen::Vector3d rpy = end_effector_state.rotation().eulerAngles(2, 1, 0);
        // ROS_INFO_STREAM("Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << "\n");
        std::double_t rollleft, rollright, pitchleft, pitchright, yawleft, yawright; 
        ros::NodeHandle nh;
        nh.param("end_effector_pose/roll_left", rollleft, std::double_t(-3.15));
        nh.param("end_effector_pose/roll_right", rollright, std::double_t(-2.4));
        nh.param("end_effector_pose/pitch_left", pitchleft, std::double_t(-0.6));
        nh.param("end_effector_pose/pitch_right", pitchright, std::double_t(0.5));
        nh.param("end_effector_pose/yaw_left", yawleft, std::double_t(-3.15));
        nh.param("end_effector_pose/yaw_right", yawright, std::double_t(-2));

        bool pitch_within_range = (pitchleft < pitch && pitch < pitchright);
        bool roll_within_range = (rollleft < roll && roll < rollright) || (-rollright < roll && roll < -rollleft);
        bool yaw_within_range = (yawleft < yaw && yaw < yawright) || (-yawright < yaw && yaw < -yawleft);

        if (!(pitch_within_range && roll_within_range && yaw_within_range)) 
        {
            all_within_range = false;
            ROS_INFO_STREAM( "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << "\n");
        }
    }

    std_msgs::Header header_out;
    header_out.stamp = ros::Time::now(); 
    header_out.frame_id = (all_within_range) ? "yes" : "no";

    end_effector_pose_pub.publish(header_out);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "end_effector_pose");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    double initialization_timeout = 30.0; // 30秒超时时间
    ros::Time start_time = ros::Time::now(); // 记录开始时间
    while(Initialization)
    {
        if ((ros::Time::now() - start_time).toSec() > initialization_timeout)
        {
            ROS_WARN("Initialization timeout. Reloading robot model.");
            robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
            const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
            start_time = ros::Time::now(); // 重新记录开始时间
        }
        std::int32_t kuka;
        nh.param("end_effector_pose/kuka", kuka, std::int32_t(3100));
        if (kuka == 3100)
        {
            // kr3100
            kinematic_state = std::make_shared<moveit::core::RobotState>(kinematic_model);
            const std::vector<double> test = {-1.76975737, -1.38124517, 
                1.10524533, 0.04357761, 1.50930967, -1.72917651};
            kinematic_state->setJointGroupPositions("leapting_arm", test);
            const Eigen::Isometry3d& end_effector_test = kinematic_state->getGlobalLinkTransform("link_6");
            Eigen::Matrix3d rotation_matrix = end_effector_test.rotation();
            Eigen::Quaterniond eigen_quaternion(rotation_matrix);
            tf::Quaternion tf_quaternion(eigen_quaternion.x(), eigen_quaternion.y(), eigen_quaternion.z(), eigen_quaternion.w());
            double rol, pit, yaw;
            tf::Matrix3x3(tf_quaternion).getRPY(rol, pit, yaw);
            double roundeRoll = std::round(rol * 10.0) / 10.0;
            double roundePitch = std::round(pit * 10.0) / 10.0;
            double roundeYaw = std::round(yaw * 10.0) / 10.0;
            ROS_INFO_STREAM( "roundeRoll: " << roundeRoll << ", roundePitch: " << roundePitch << ", roundeYaw: " << roundeYaw << "\n");
            if(roundeRoll == -2.8 && roundePitch == 0.0 && roundeYaw == -3.1)
            {
                Initialization = false;
                ROS_INFO_STREAM("end effector pose Initialization ok.");
            }
        }

        else
        {
            // kr2700
            kinematic_state = std::make_shared<moveit::core::RobotState>(kinematic_model);
            const std::vector<double> test = {-1.6182002649758722, -1.579311254210773, 
                        1.1212306552173565, 0.006935875196446339, 1.6236686987032538, -1.6604176361856133};
            kinematic_state->setJointGroupPositions("leapting_arm", test);
            const Eigen::Isometry3d& end_effector_test = kinematic_state->getGlobalLinkTransform("link_6");
            Eigen::Matrix3d rotation_matrix = end_effector_test.rotation();
            Eigen::Quaterniond eigen_quaternion(rotation_matrix);
            tf::Quaternion tf_quaternion(eigen_quaternion.x(), eigen_quaternion.y(), eigen_quaternion.z(), eigen_quaternion.w());
            double rol, pit, yaw;
            tf::Matrix3x3(tf_quaternion).getRPY(rol, pit, yaw);
            double roundeRoll = std::round(rol * 10.0) / 10.0;
            double roundePitch = std::round(pit * 10.0) / 10.0;
            double roundeYaw = std::round(yaw * 10.0) / 10.0;
            ROS_INFO_STREAM( "roundeRoll: " << roundeRoll << ", roundePitch: " << roundePitch << ", roundeYaw: " << roundeYaw << "\n");
            if(roundeRoll == -2.7 && roundePitch == 0.0 && roundeYaw == 3.1)
            {
                Initialization = false;
                ROS_INFO_STREAM("end effector pose Initialization ok.");
            }
        }
        
        // Introduce a 1-second delay before the next iteration
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    ros::Subscriber cmd_sub = nh.subscribe("joint_trajectory_todo", 10, PoseCallback);
    end_effector_pose_pub = nh.advertise<std_msgs::Header>("joint_trajectory_done", 10);
    ros::spin();
    return 0;

}