#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class Path_to_goal(object):

    def __init__(self):
        rospy.init_node("path_to_goal_node")
        rospy.Subscriber('/odom', Odometry, self.callback)
        rospy.Subscriber("path_topic", Path, self.path_callback)
        self.position_list = []
        self.goals = []
        self.previous_goal = None
        self.goal_publisher = rospy.Publisher(
            "move_base_simple/goal", PoseStamped, latch=True, queue_size=10)

    def callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        self.position_list.append([x, y, z])
        rospy.loginfo("小车位置：(x={}, y={}, z={})".format(x, y, z))

    def path_callback(self, path):
        num_segments = int(self.calculate_path_length(path))  # 将路径平均分成n段
        self.current_goal_index = 0
        self.goals = self.extract_goals(path, num_segments)

    def publish_goal(self):
        if len(self.position_list) > 0:
            current_position = self.position_list[-1]
            goal = self.get_goal(current_position, self.goals)
            if goal is None:
                return
            else:
                if goal != self.previous_goal:
                    self.goal_publisher.publish(goal)
                    print('pub once')
                    self.previous_goal = goal
        else:
            # 处理位置信息为空的情况
            rospy.loginfo("wait pose")

    def calculate_path_length(self, path):
        total_distance = 0.0
        for i in range(len(path.poses) - 1):
            pose1 = path.poses[i].pose
            pose2 = path.poses[i + 1].pose

            distance = ((pose2.position.x - pose1.position.x) ** 2 +
                        (pose2.position.y - pose1.position.y) ** 2 +
                        (pose2.position.z - pose1.position.z) ** 2) ** 0.5

            total_distance += distance
        return total_distance

    def extract_goals(self, path, num_segments):
        self.goals = []
        segment_length = len(path.poses) // num_segments
        for i in range(num_segments):
            index = (i + 1) * segment_length
            pose = path.poses[index].pose

            goal = PoseStamped()
            goal.header.frame_id = path.header.frame_id
            goal.pose.position.x = pose.position.x
            goal.pose.position.y = pose.position.y
            goal.pose.position.z = pose.position.z
            goal.pose.orientation = pose.orientation

            self.goals.append(goal)

        rospy.loginfo(self.goals)

        return self.goals

    def get_goal(self, current_position, goals):
        # 检查当前目标是否已经达到
        if self.goals==[]:
            return
         
        current_goal = self.goals[self.current_goal_index]


        if self.distance(current_goal, current_position) < 0.4:
            # 当前目标已经达到，更新当前目标为下一个点
            if self.current_goal_index >= len(goals)-1:
                # 已经到达路径的最后一个点
                return None
            else:
                self.current_goal_index += 1
                return self.goals[self.current_goal_index]
        else:
            # 返回当前目标
            return current_goal


    def distance(self, point1, point2):
        # 计算两点之间的距离
        return ((point1.pose.position.x - point2[0])**2
                + (point1.pose.position.y - point2[1])**2
                + (point1.pose.position.z - point2[2])**2)**0.5


if __name__ == '__main__':
    node = Path_to_goal()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        node.publish_goal()
        rate.sleep()
    rospy.spin()
