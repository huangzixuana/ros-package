#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped


class Path_to_goal(object):
   
    def __init__(self):
        rospy.init_node("path_to_goal_node")
        rospy.Subscriber('robot_pose', Pose, self.callback)        #订阅小车位置话题
        rospy.Subscriber("path_topic", Path,                       #订阅path话题
                    lambda msg: self.local_plan_callback(msg, self.tf_buffer))
        self.position_list = []
        self.goals = []
        self.previous_goal = None
        self.goal_publisher = rospy.Publisher(
            "goal", PoseStamped, latch=True, queue_size=10)        #发布goal话题
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))  # 设置缓冲区的时长
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def convert_pose(self, pose, tf_buffer):
        try:
        # 从base_laser坐标系到map坐标系的转换关系
            transform = tf_buffer.lookup_transform("map", pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
            return transformed_pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to transform pose")
            return None
        
    def local_plan_callback(self, msg, tf_buffer):
        # 将base_laser下的path转换到map坐标系下，并提取goal
        transformed_plan = Path()
        transformed_plan.header = msg.header
        transformed_plan.header.frame_id = "map"
        for pose in msg.poses:
            transformed_pose = self.convert_pose(pose, tf_buffer)
            if transformed_pose:
                transformed_plan.poses.append(transformed_pose)
        num_segments = int(self.calculate_path_length(transformed_plan))
        self.current_goal_index = 0
        self.goals = self.extract_goals(transformed_plan, num_segments)

    def callback(self, date):
        #获得小车位置
        x = date.position.x
        y = date.position.y
        z = date.position.z
        self.position_list.append([x, y, z])
        #rospy.loginfo("小车位置：(x={}, y={}, z={})".format(x, y, z))

    def publish_goal(self):
        #发布goal
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
        #计算path长度
        total_distance = 0.0
        for i in range(len(path.poses) - 1):
            pose1 = path.poses[i].pose
            pose2 = path.poses[i + 1].pose
            distance = ((pose2.position.x - pose1.position.x) ** 2 +
                        (pose2.position.y - pose1.position.y) ** 2 +
                        (pose2.position.z - pose1.position.z) ** 2) ** 0.5
            total_distance += distance
        return total_distance

    def extract_goals(self, transformed_plan, num_segments):
        #每隔1m提取一个goal，并放入一个列表中
        self.goals = []
        segment_length = len(transformed_plan.poses) // num_segments
        for i in range(num_segments):
            index = (i + 1) * segment_length
            pose = transformed_plan.poses[index].pose
            goal = PoseStamped()
            goal.header.frame_id = transformed_plan.header.frame_id
            goal.pose.position.x = pose.position.x
            goal.pose.position.y = pose.position.y
            goal.pose.position.z = pose.position.z
            goal.pose.orientation = pose.orientation
            self.goals.append(goal)
        return self.goals

    def get_goal(self, current_position, goals):
        # 检查当前目标是否已经达到
        if self.goals==[]:
            return   
        current_goal = self.goals[self.current_goal_index]
        if self.distance(current_goal, current_position) < 0.5:
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
