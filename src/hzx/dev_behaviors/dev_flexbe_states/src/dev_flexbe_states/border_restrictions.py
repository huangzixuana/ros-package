#!/usr/bin/env python
import tf
import math
import moveit_commander
import geometry_msgs.msg
from flexbe_core import EventState, Logger

'''
Updated on Oct 19st, 2023
@author: zixuan.huang@leapting.com
'''

class BorderRestrictions(EventState):
    '''
    A state to add or remove border restrictions for a robot.

    -- action  	            string          "add" or "remove".
    -- cube_size	        list            [x, y, z], unit is meter
    -- frame_id  	        string          reference_frame of cube
    -- position_x           float           The distance of x between the plane0 and the frame_id
    -- position_y           float           The distance of y between the plane0 and the frame_id
    -- position_z           float           The distance of z between the plane0 and the frame_id

    <= done		    action complete.
    '''

    def __init__(self, action="add",
                 cube_size=[5.6, 5, 3],
                 frame_id="base_arm",
                 position_x=0.8,
                 position_y=-0.9,
                 position_z=2.9):
        super(BorderRestrictions, self).__init__(outcomes=['done'])
        self._action = action
        self._cube_size = cube_size
        self._frame_id = frame_id
        self._position_x = position_x
        self._position_y = position_y
        self._position_z = position_z
        self.scene = moveit_commander.PlanningSceneInterface()
        self.thickness = 0.001
        self.plane_pose = [geometry_msgs.msg.PoseStamped() for _ in range(6)]
        
    def execute(self, userdata):

        if self._action == "add":
            self.scene.add_box("plane0", self.plane_pose[0], size=(self._cube_size[0],self._cube_size[1],self.thickness))
            if ("plane0" not in self.scene.get_known_object_names()):
                return      
            self.scene.add_box("plane1", self.plane_pose[1], size=(self._cube_size[0],self._cube_size[1],self.thickness))
            if ("plane1" not in self.scene.get_known_object_names()):
                return  
            self.scene.add_box("plane2", self.plane_pose[2], size=(self._cube_size[2],self._cube_size[0],self.thickness))
            if ("plane2" not in self.scene.get_known_object_names()):
                return  
            self.scene.add_box("plane3", self.plane_pose[3], size=(self._cube_size[2],self._cube_size[0],self.thickness))
            if ("plane3" not in self.scene.get_known_object_names()):
                return  
            self.scene.add_box("plane4", self.plane_pose[4], size=(self._cube_size[1],self._cube_size[2],self.thickness))
            if ("plane4" not in self.scene.get_known_object_names()):
                return  
            self.scene.add_box("plane5", self.plane_pose[5], size=(self._cube_size[1],self._cube_size[2],self.thickness))
            if ("plane5" not in self.scene.get_known_object_names()):
                return  
            Logger.loginfo('add succeed: executed')
            return 'done'
            
        else:
            self.scene.remove_world_object()
            if (len(self.scene.get_known_object_names())>0):
                return 
            Logger.loginfo('remove succeed: executed')
            return 'done'

    def on_enter(self, userdata):

        self.plane_pose[0].header.frame_id = self._frame_id
        self.plane_pose[0].pose.position.x = self._position_x
        self.plane_pose[0].pose.position.y = self._position_y
        self.plane_pose[0].pose.position.z = self._position_z
        
        self.plane_pose[1].header.frame_id = self._frame_id
        self.plane_pose[1].pose.position.x = self._position_x
        self.plane_pose[1].pose.position.y = self._position_y
        self.plane_pose[1].pose.position.z = self._position_z-self._cube_size[2]

        quaternion3 = tf.transformations.quaternion_from_euler(0, math.pi/2, math.pi/2)

        self.plane_pose[2].header.frame_id = self._frame_id
        self.plane_pose[2].pose.position.x = self._position_x
        self.plane_pose[2].pose.position.y = self._cube_size[1]/2+self._position_y
        self.plane_pose[2].pose.position.z = self._position_z-self._cube_size[2]/2
        self.plane_pose[2].pose.orientation.x = quaternion3[0]
        self.plane_pose[2].pose.orientation.y = quaternion3[1]
        self.plane_pose[2].pose.orientation.z = quaternion3[2]
        self.plane_pose[2].pose.orientation.w = quaternion3[3]
        
        self.plane_pose[3].header.frame_id = self._frame_id
        self.plane_pose[3].pose.position.x = self._position_x
        self.plane_pose[3].pose.position.y = self._cube_size[1]/2+self._position_y-self._cube_size[1]
        self.plane_pose[3].pose.position.z = self._position_z-self._cube_size[2]/2
        self.plane_pose[3].pose.orientation.x = quaternion3[0]
        self.plane_pose[3].pose.orientation.y = quaternion3[1]
        self.plane_pose[3].pose.orientation.z = quaternion3[2]
        self.plane_pose[3].pose.orientation.w = quaternion3[3]

        quaternion5 = tf.transformations.quaternion_from_euler(math.pi/2, 0, math.pi/2)

        self.plane_pose[4].header.frame_id = self._frame_id
        self.plane_pose[4].pose.position.x = self._cube_size[0]/2+self._position_x
        self.plane_pose[4].pose.position.y = self._position_y
        self.plane_pose[4].pose.position.z = self._position_z-self._cube_size[2]/2
        self.plane_pose[4].pose.orientation.x = quaternion5[0]
        self.plane_pose[4].pose.orientation.y = quaternion5[1]
        self.plane_pose[4].pose.orientation.z = quaternion5[2]
        self.plane_pose[4].pose.orientation.w = quaternion5[3]

        self.plane_pose[5].header.frame_id = self._frame_id
        self.plane_pose[5].pose.position.x = self._cube_size[0]/2+self._position_x-self._cube_size[0]
        self.plane_pose[5].pose.position.y = self._position_y
        self.plane_pose[5].pose.position.z = self._position_z-self._cube_size[2]/2
        self.plane_pose[5].pose.orientation.x = quaternion5[0]
        self.plane_pose[5].pose.orientation.y = quaternion5[1]
        self.plane_pose[5].pose.orientation.z = quaternion5[2]
        self.plane_pose[5].pose.orientation.w = quaternion5[3]

