#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path

def save_path_to_file(path_msg, file_path):
    with open(file_path, 'w') as file:
        for pose in path_msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            z = pose.pose.position.z
            qx = pose.pose.orientation.x
            qy = pose.pose.orientation.y
            qz = pose.pose.orientation.z
            qw = pose.pose.orientation.w
            file.write("{}, {}, {}, {}, {}, {}, {}\n".format(x, y, z, qx, qy, qz, qw))

def extract_path_and_save():
    # 创建 ROS 节点
    rospy.init_node('path_extractor', anonymous=True)

    # 订阅路径话题，即 teb_local_planner 发布的路径
    file_path = '/home/hzx/path/path.txt'  # 修改为实际的文件路径
    rospy.Subscriber('/move_base/TebLocalPlannerROS/local_plan', Path, callback=lambda msg: save_path_to_file(msg, file_path))

    # 等待路径消息到达
    rospy.spin()

if __name__ == '__main__':
    try:
        extract_path_and_save()
    except rospy.ROSInterruptException:
        pass