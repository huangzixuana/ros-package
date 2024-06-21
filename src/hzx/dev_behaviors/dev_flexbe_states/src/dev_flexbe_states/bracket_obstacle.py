#!/usr/bin/env python
import sys
import rospy
import rostopic
import tf
import math
import moveit_commander
import geometry_msgs.msg
from flexbe_core import EventState, Logger
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
    -- box_position         list            The position between the bracke and the frame_id, unit is meter
    -- box_orientation      list            The orientation between the bracke and the frame_id

    <= done		    action complete.
    '''

    def __init__(self, bracke="add",
                 obstacle="add",
                 object_size=[2.278, 1.134, 0.035],
                 frame_id="base_arm",
                 box_position=[0, 0, 0],
                 box_orientation=[0, 0, 0 ,1]):
        super(BrackeObstacle, self).__init__(outcomes=['done'])
        moveit_commander.roscpp_initialize(sys.argv)
        self._bracke = bracke
        self._obstacle = obstacle
        self._size = object_size
        self._position = box_position
        self._orientation = box_orientation
        self.eef_link = frame_id
        self.box_pose = geometry_msgs.msg.PoseStamped()
        self.plane_pose = [geometry_msgs.msg.PoseStamped() for _ in range(4)]
        
    def execute(self, userdata):

        msg_type, msg_topic, _ = rostopic.get_topic_class('/move_group/goal')
        if msg_topic is None:
            Logger.logwarn('Moveit Failed')
            return
        else:
            self.scene = moveit_commander.PlanningSceneInterface()

        if self._bracke == "add":
            self.scene.add_box("bracke", self.box_pose, size=(self._size[0],self._size[1], self._size[2]))
            if ("bracke" not in self.scene.get_known_object_names()):
                return      
            Logger.loginfo('BrackeObstacle add bracke succeed: executed')
        else:
            self.scene.remove_world_object()
            Logger.loginfo('BrackeObstacle remove bracke succeed: executed')

        if self._obstacle == "add":
            # self.scene.add_box("obstacle0", self.plane_pose[0], size=(0.8,1.6,0.01))
            # if ("obstacle0" not in self.scene.get_known_object_names()):
            #     return      
            self.scene.add_box("obstacle1", self.plane_pose[1], size=(0.8,1.6,0.01))
            if ("obstacle1" not in self.scene.get_known_object_names()):
                return  
            # self.scene.add_box("obstacle2", self.plane_pose[2], size=(2.6,0.8,0.01))
            self.scene.add_box("obstacle2", self.plane_pose[2], size=(0.8,0.8,0.01))
            if ("obstacle2" not in self.scene.get_known_object_names()):
                return  
            self.scene.add_box("obstacle3", self.plane_pose[3], size=(2.6,0.4,0.01))
            if ("obstacle3" not in self.scene.get_known_object_names()):
                return  
            Logger.loginfo('BrackeObstacle add obstacle succeed: executed')
            return 'done'
        else:
            self.scene.remove_world_object("obstacle0")
            self.scene.remove_world_object("obstacle1")
            self.scene.remove_world_object("obstacle2")
            self.scene.remove_world_object("obstacle3")
            if ("obstacle0" in self.scene.get_known_object_names()):
                return      
            if ("obstacle1" in self.scene.get_known_object_names()):
                return  
            if ("obstacle2" in self.scene.get_known_object_names()):
                return  
            if ("obstacle3" in self.scene.get_known_object_names()):
                return
            Logger.loginfo('BrackeObstacle remove obstacle succeed: executed')
            return 'done'

    def on_enter(self, userdata):
        self.box_pose.header.frame_id = self.eef_link
        self.box_pose.pose.position.x = self._position[0] 
        self.box_pose.pose.position.y = self._position[1]
        self.box_pose.pose.position.z = self._position[2]
        self.box_pose.pose.orientation.x = self._orientation[0] 
        self.box_pose.pose.orientation.y = self._orientation[1]
        self.box_pose.pose.orientation.z = self._orientation[2]
        self.box_pose.pose.orientation.w = self._orientation[3]

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
        # self.plane_pose[2].pose.position.x = 2.989
        self.plane_pose[2].pose.position.x = 2.789
        # self.plane_pose[2].pose.position.y = -0.035
        self.plane_pose[2].pose.position.y = -0.835
        self.plane_pose[2].pose.position.z = 0.064
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
