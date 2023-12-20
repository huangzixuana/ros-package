#!/usr/bin/env python3
import moveit_commander
import geometry_msgs.msg
import tf
import math

cube_size=[5, 4.5, 4.8, 0.00001]
position_x=0
position_y=1.0
position_z=3.6

scene = moveit_commander.PlanningSceneInterface()

plane1_pose = geometry_msgs.msg.PoseStamped()
plane1_pose.header.frame_id = "base_arm"
plane1_pose.pose.position.x = position_x
plane1_pose.pose.position.y = position_y
plane1_pose.pose.position.z = position_z
scene.add_box("plane1", plane1_pose, size=(cube_size[0],cube_size[1],cube_size[3]))

plane2_pose = geometry_msgs.msg.PoseStamped()
plane2_pose.header.frame_id = "base_arm"
plane2_pose.pose.position.x = position_x
plane2_pose.pose.position.y = position_y
plane2_pose.pose.position.z = position_z-cube_size[2]
scene.add_box("plane2", plane2_pose, size=(cube_size[0],cube_size[1],cube_size[3]))

quaternion3 = tf.transformations.quaternion_from_euler(0, math.pi/2, math.pi/2)

plane3_pose = geometry_msgs.msg.PoseStamped()
plane3_pose.header.frame_id = "base_arm"
plane3_pose.pose.position.x = position_x
plane3_pose.pose.position.y = cube_size[1]/2+position_y
plane3_pose.pose.position.z = position_z-cube_size[2]/2
plane3_pose.pose.orientation.x = quaternion3[0]
plane3_pose.pose.orientation.y = quaternion3[1]
plane3_pose.pose.orientation.z = quaternion3[2]
plane3_pose.pose.orientation.w = quaternion3[3]
scene.add_box("plane3", plane3_pose, size=(cube_size[2],cube_size[0],cube_size[3]))

plane4_pose = geometry_msgs.msg.PoseStamped()
plane4_pose.header.frame_id = "base_arm"
plane4_pose.pose.position.x = position_x
plane4_pose.pose.position.y = cube_size[1]/2+position_y-cube_size[1]
plane4_pose.pose.position.z = position_z-cube_size[2]/2
plane4_pose.pose.orientation.x = quaternion3[0]
plane4_pose.pose.orientation.y = quaternion3[1]
plane4_pose.pose.orientation.z = quaternion3[2]
plane4_pose.pose.orientation.w = quaternion3[3]
scene.add_box("plane4", plane4_pose, size=(cube_size[2],cube_size[0],cube_size[3]))

quaternion5 = tf.transformations.quaternion_from_euler(math.pi/2, 0, math.pi/2)

plane5_pose = geometry_msgs.msg.PoseStamped()
plane5_pose.header.frame_id = "base_arm"
plane5_pose.pose.position.x = cube_size[0]/2+position_x
plane5_pose.pose.position.y = position_y
plane5_pose.pose.position.z = position_z-cube_size[2]/2
plane5_pose.pose.orientation.x = quaternion5[0]
plane5_pose.pose.orientation.y = quaternion5[1]
plane5_pose.pose.orientation.z = quaternion5[2]
plane5_pose.pose.orientation.w = quaternion5[3]
scene.add_box("plane5", plane5_pose, size=(cube_size[1],cube_size[2],cube_size[3]))

plane6_pose = geometry_msgs.msg.PoseStamped()
plane6_pose.header.frame_id = "base_arm"
plane6_pose.pose.position.x = cube_size[0]/2+position_x-cube_size[0]
plane6_pose.pose.position.y = position_y
plane6_pose.pose.position.z = position_z-cube_size[2]/2
plane6_pose.pose.orientation.x = quaternion5[0]
plane6_pose.pose.orientation.y = quaternion5[1]
plane6_pose.pose.orientation.z = quaternion5[2]
plane6_pose.pose.orientation.w = quaternion5[3]
scene.add_box("plane6", plane6_pose, size=(cube_size[1],cube_size[2],cube_size[3]))

scene.remove_world_object()
