#!/usr/bin/env python
import sys
import rospy
import rostopic
import moveit_commander
import geometry_msgs.msg
from flexbe_core import EventState, Logger
'''
Updated on Oct 19st, 2023
@author: zixuan.huang@leapting.com
'''

class BrackeObstacle(EventState):
    '''
    A state to add or remove bracke obstacle.

    -- action  	            string          "add" or "remove".
    -- cube_size	        list            [x, y, z], unit is meter
    -- frame_id  	        string          reference_frame of cube
    -- position_x           float           The distance of x between the plane0 and the frame_id
    -- position_y           float           The distance of y between the plane0 and the frame_id
    -- position_z           float           The distance of z between the plane0 and the frame_id

    <= done		    action complete.
    '''

    def __init__(self, action="add",
                 object_size=[2.278, 1.134, 0.035],
                 frame_id="base_arm",
                 box_position=[0, 0, 0],
                 box_orientation=[0, 0, 0 ,1]):
        super(BrackeObstacle, self).__init__(outcomes=['done'])
        moveit_commander.roscpp_initialize(sys.argv)
        self._action = action
        self._size = object_size
        self._position = box_position
        self._orientation = box_orientation
        self.eef_link = frame_id
        self.box_pose = geometry_msgs.msg.PoseStamped()
        
    def execute(self, userdata):

        msg_type, msg_topic, _ = rostopic.get_topic_class('/move_group/goal')
        if msg_topic is None:
            Logger.logwarn('Moveit Failed')
            return
        else:
            self.scene = moveit_commander.PlanningSceneInterface()

        if self._action == "add":
            self.scene.add_box("bracke", self.box_pose, size=(self._size[0],self._size[1], self._size[2]))
            if ("bracke" not in self.scene.get_known_object_names()):
                return      
            Logger.loginfo('add succeed: executed')
            return 'done'
            
        else:
            self.scene.remove_world_object()
            Logger.loginfo('remove succeed: executed')
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