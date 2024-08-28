#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_model_and_robot_state_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();

  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");

  const std::vector<double> gstate = {1.0, 1.0, 1.0, 1.0, 2.0, 1.0, 1.0};
  kinematic_state->setJointGroupPositions("panda_arm", gstate);
  // kinematic_state->setToRandomPositions(joint_model_group);
  const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("panda_link8");

  ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
  ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

  ros::shutdown();
  return 0;
}
