#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def publish_path():
    rospy.init_node('path_publisher')

    # 创建路径消息
    path_msg = Path()

    # 设置路径消息的header信息
    path_msg.header.frame_id = "map"
    path_msg.header.stamp = rospy.Time.now()

    # 从文件中读取保存的路径
    file_path = '/home/hzx/catkin_ws/src/path_to_goal/scripts/poses.txt'
    with open(file_path, 'r') as f:
        for line in f:
            pose_data = line.rstrip().split(' ')
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = float(pose_data[0])
            pose.pose.position.y = float(pose_data[1])
            pose.pose.position.z = float(pose_data[2])
            pose.pose.orientation.x = float(pose_data[3])
            pose.pose.orientation.y = float(pose_data[4])
            pose.pose.orientation.z = float(pose_data[5])
            pose.pose.orientation.w = float(pose_data[6])
            path_msg.poses.append(pose)

    # 创建路径发布者
    pub = rospy.Publisher('path_topic', Path, queue_size=10,latch=True)

    # 发布路径消息
    pub.publish(path_msg)

    while not rospy.is_shutdown():
        pass

if __name__ == '__main__':
    try:
        publish_path()
    except rospy.ROSInterruptException:
        pass