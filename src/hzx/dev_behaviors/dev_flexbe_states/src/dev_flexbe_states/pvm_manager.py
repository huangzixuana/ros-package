#!/usr/bin/env python
import sys
import rospy
import rostopic
import moveit_commander
import geometry_msgs.msg
from flexbe_core import EventState, Logger

'''
Updated on Oct 17st, 2023
@author: zixuan.huang@leapting.com
'''


class PvmManager(EventState):
    '''
    A state that can attach a pvm to the robot and detach the pvm from the robot.

    -- action  	            string          "attach" or "detach".
    -- pvm_size	            list            [x, y, z], unit is meter
    -- frame_id  	        string          reference_frame of pvm
    -- position_z           float           The distance of z between the pvm and the frame_id

    <= done		    action complete.
    '''

    def __init__(self, action="attach",
                 pvm_size=[2.278, 1.134, 0.035],
                 frame_id="tool0",
                 position_z=0):
        super(PvmManager, self).__init__(outcomes=['done'])
        moveit_commander.roscpp_initialize(sys.argv)
        self._action = action
        self._size = pvm_size
        self._frame_id = frame_id
        self._position_z = position_z
        self.pvm_name = "pvm"
        self.eef_link = frame_id
        self.pvm_pose = geometry_msgs.msg.PoseStamped()
        
    def execute(self, userdata):
        
        msg_type, msg_topic, _ = rostopic.get_topic_class('/move_group/goal')
        if msg_topic is None:
            Logger.logwarn('Moveit Failed')
            return
        else:
            self.scene = moveit_commander.PlanningSceneInterface()

        if self._action == "attach":
            self.scene.add_box(self.pvm_name, self.pvm_pose, size=(self._size[0],self._size[1], self._size[2]))
            if not (self.wait_for_state_update(box_is_known=True) or 
                    self.wait_for_state_update(box_is_known=True,box_is_attached=True)):
                return      
            self.scene.attach_box(self.eef_link, self.pvm_name, touch_links=[])
            if not self.wait_for_state_update(box_is_attached=True):
                return
            Logger.loginfo('attach succeed: executed')
            return 'done'
            
        else:
            self.scene.remove_attached_object(self.eef_link, name=self.pvm_name)
            if not (self.wait_for_state_update(box_is_known=True) or 
                    self.wait_for_state_update()):
                return
            self.scene.remove_world_object(self.pvm_name)
            if not self.wait_for_state_update():
                return
            Logger.loginfo('detach succeed: executed')
            return 'done'


    def on_enter(self, userdata):
        if rospy.has_param('robot_state/pvm_length'):
            self._size[0] = rospy.get_param('robot_state/pvm_length') * 0.001
        if rospy.has_param('robot_state/pvm_width'):
            self._size[1] = rospy.get_param('robot_state/pvm_width') * 0.001
        self.pvm_pose.header.frame_id = self._frame_id
        self.pvm_pose.pose.position.z = self._position_z 

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False):
        attached_objects = self.scene.get_attached_objects([self.pvm_name])
        is_attached = len(attached_objects.keys()) > 0
        is_known = self.pvm_name in self.scene.get_known_object_names()
        if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True
        else:
            return False
