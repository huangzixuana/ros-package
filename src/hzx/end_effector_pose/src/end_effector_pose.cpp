#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "end_effector_pose.cpp");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();

    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("leapting_arm");

    // const std::vector<double> gstate = {-1.6231562043547263, -1.5707963267948966, 1.117010721276371, 0.0, 1.6231562043547263, -1.6580627893946132};
    const std::vector<double> gstate = {0.03490658503988659, -1.780235837034216, 1.7453292519943295, -0.017453292519943295, 1.5009831567151233, 0.017453292519943295};
    // const std::vector<double> gstate = {0.0, -1.5882496193148399, 0.9773843811168246, 0.0, 0.0, 0.0};
    kinematic_state->setJointGroupPositions("leapting_arm", gstate);
    const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("link_6");
    ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
    ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");
    Eigen::Vector3d rpy = end_effector_state.rotation().eulerAngles(0, 1, 2);
    ROS_INFO_STREAM("Roll: " << rpy(2) << ", Pitch: " << rpy(1) << ", Yaw: " << rpy(0) << "\n");

    ros::shutdown();
    return 0;
}
