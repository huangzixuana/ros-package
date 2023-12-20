#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

def convert_pose(pose, tf_buffer):
    try:
        # 从map坐标系到odom坐标系的转换关系
        transform = tf_buffer.lookup_transform("odom", pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
        return transformed_pose
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn("Failed to transform pose")
        return None

def local_plan_callback(msg, tf_buffer, pub):
    transformed_plan = Path()
    transformed_plan.header = msg.header

    for pose in msg.poses:
        transformed_pose = convert_pose(pose, tf_buffer)
        if transformed_pose:
            transformed_plan.poses.append(transformed_pose)
    
    # 发布转换后的路径到新的话题
    pub.publish(transformed_plan)

def main():
    rospy.init_node("plan_converter")

    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))  # 设置缓冲区的时长
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # 创建一个Publisher发布新的路径话题
    pub = rospy.Publisher("/odom_local_plan", Path, queue_size=10)

    # 订阅原始路径话题
    rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan", Path, 
                     lambda msg: local_plan_callback(msg, tf_buffer, pub))

    rospy.spin()

if __name__ == '__main__':
    main()