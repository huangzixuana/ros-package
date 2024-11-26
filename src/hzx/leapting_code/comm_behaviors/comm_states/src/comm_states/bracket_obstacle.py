#!/usr/bin/env python
import sys
import rospy
import rostopic
import tf
import math
import moveit_commander
import geometry_msgs.msg
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

'''
Updated on Jun 5st, 2024
@author: zixuan.huang@leapting.com
'''

class BrackeObstacle(EventState):
    '''
    A state to add or remove bracke obstacle.

    -- bracke  	            string          "add" or "remove".
    -- obstacle  	        string          "add" or "remove".
    -- object_size	        list            [x, y, z], unit is meter
    -- frame_id  	        string          reference_frame of obstacle
    -- obstacle_position    list            The position between the obstacle and the frame_id, unit is meter
    -- obstacle_size        list            [x, y, z], unit is meter

    <= done		    action complete.
    '''

    def __init__(self, bracke="add",
                 obstacle="add",
                 bracke_size=[2.338, 10, 0.1],
                 frame_id="base_arm",
                 obstacle_position=[2.789, -0.835, 0.34],
                 obstacle_size=[0.6, 1.5, 0.01]):
        super(BrackeObstacle, self).__init__(outcomes=['done'])
        moveit_commander.roscpp_initialize(sys.argv)
        self._bracke = bracke
        self._obstacle = obstacle
        self._size = bracke_size
        self._position = obstacle_position
        self._obstacle_size = obstacle_size
        self.eef_link = frame_id
        self.box_pose = geometry_msgs.msg.PoseStamped()
        self.plane_pose = [geometry_msgs.msg.PoseStamped() for _ in range(4)]

        self._solar_msg = None
        self._received = False
        self._solar_sub = ProxySubscriberCached(
            {'filter_solar_pose': PoseWithCovarianceStamped})
        self._solar_sub.subscribe('filter_solar_pose', PoseWithCovarianceStamped,
                                  callback=self.solar_cb, buffered=False)
        # current_time = time.time()
        # Logger.loginfo("当前时间是: {0}".format(current_time))

    def solar_cb(self, msg):
        self._solar_msg = msg
        self._received = True

    def on_enter(self, userdata):
        self._received = False

        quaternion1 = tf.transformations.quaternion_from_euler(0, math.pi/2, math.pi/2)
        quaternion2 = tf.transformations.quaternion_from_euler(math.pi/2, 0, math.pi/2)

        self.plane_pose[0].header.frame_id = self.eef_link
        self.plane_pose[0].pose.position.x = 2.089
        self.plane_pose[0].pose.position.y = -1.335
        self.plane_pose[0].pose.position.z = 0.044
        self.plane_pose[0].pose.orientation.x = quaternion1[0]
        self.plane_pose[0].pose.orientation.y = quaternion1[1]
        self.plane_pose[0].pose.orientation.z = quaternion1[2]
        self.plane_pose[0].pose.orientation.w = quaternion1[3]

        self.plane_pose[1].header.frame_id = self.eef_link
        self.plane_pose[1].pose.position.x = 2.089
        self.plane_pose[1].pose.position.y = 1.265
        self.plane_pose[1].pose.position.z = 0.044
        self.plane_pose[1].pose.orientation.x = quaternion1[0]
        self.plane_pose[1].pose.orientation.y = quaternion1[1]
        self.plane_pose[1].pose.orientation.z = quaternion1[2]
        self.plane_pose[1].pose.orientation.w = quaternion1[3]

        self.plane_pose[2].header.frame_id = self.eef_link
        self.plane_pose[2].pose.position.x = self._position[0]
        self.plane_pose[2].pose.position.y = self._position[1]
        self.plane_pose[2].pose.position.z = self._position[2]
        self.plane_pose[2].pose.orientation.x = quaternion2[0]
        self.plane_pose[2].pose.orientation.y = quaternion2[1]
        self.plane_pose[2].pose.orientation.z = quaternion2[2]
        self.plane_pose[2].pose.orientation.w = quaternion2[3]
        
        self.plane_pose[3].header.frame_id = self.eef_link
        self.plane_pose[3].pose.position.x = 1.289
        self.plane_pose[3].pose.position.y = -0.035
        self.plane_pose[3].pose.position.z = 0.064
        self.plane_pose[3].pose.orientation.x = quaternion2[0]
        self.plane_pose[3].pose.orientation.y = quaternion2[1]
        self.plane_pose[3].pose.orientation.z = quaternion2[2]
        self.plane_pose[3].pose.orientation.w = quaternion2[3]

        
    def execute(self, userdata):

        msg_type, msg_topic, _ = rostopic.get_topic_class('/move_group/goal')
        if msg_topic is None:
            Logger.logwarn('Moveit Failed')
            return
        else:
            self.scene = moveit_commander.PlanningSceneInterface()

        if not self._received:
            self.box_pose.header.frame_id = self.eef_link
            self.box_pose.pose.position.x = 0.67 
            self.box_pose.pose.position.y = 2.12
            self.box_pose.pose.position.z = 0.86
            self.box_pose.pose.orientation.x = -0.0159
            self.box_pose.pose.orientation.y = 0.00081
            self.box_pose.pose.orientation.z = 0.720647
            self.box_pose.pose.orientation.w = 0.693118

        else:
            self.box_pose.header.frame_id = self.eef_link
            self.box_pose.pose.position.x = self._solar_msg.pose.pose.position.x 
            self.box_pose.pose.position.y = self._solar_msg.pose.pose.position.y
            self.box_pose.pose.position.z = self._solar_msg.pose.pose.position.z
            self.box_pose.pose.orientation.x = self._solar_msg.pose.pose.orientation.x
            self.box_pose.pose.orientation.y = self._solar_msg.pose.pose.orientation.y
            self.box_pose.pose.orientation.z = self._solar_msg.pose.pose.orientation.z
            self.box_pose.pose.orientation.w = self._solar_msg.pose.pose.orientation.w

        # current_time = time.time()
        # Logger.loginfo("当前时间是: {0}".format(current_time))
        if self._bracke == "add":
            self.scene.add_box("bracke", self.box_pose, size=(self._size[0],self._size[1], self._size[2]))
            if ("bracke" not in self.scene.get_known_object_names()):
                return      
            Logger.loginfo('BrackeObstacle add bracke succeed: executed')
        else:
            self.scene.remove_world_object()
            Logger.loginfo('BrackeObstacle remove bracke succeed: executed')
        # current_time1 =  time.time()
        # Logger.loginfo("当前时间是: {0}".format(current_time1))

        if self._obstacle == "add":
            # self.scene.add_box("obstacle0", self.plane_pose[0], size=(0.8,1.6,0.01))
            # if ("obstacle0" not in self.scene.get_known_object_names()):
            #     return      
            self.scene.add_box("obstacle1", self.plane_pose[1], size=(0.8,1.6,0.01))
            if ("obstacle1" not in self.scene.get_known_object_names()):
                return  
            # self.scene.add_box("obstacle2", self.plane_pose[2], size=(2.6,0.8,0.01))
            self.scene.add_box("obstacle2", self.plane_pose[2], size
                               =(self._obstacle_size[0],self._obstacle_size[1],self._obstacle_size[2]))
            if ("obstacle2" not in self.scene.get_known_object_names()):
                return  
            self.scene.add_box("obstacle3", self.plane_pose[3], size=(2.6,0.4,0.01))
            if ("obstacle3" not in self.scene.get_known_object_names()):
                return  
            Logger.loginfo('BrackeObstacle add obstacle succeed: executed')
            return 'done'
        else:
            return 'done'
