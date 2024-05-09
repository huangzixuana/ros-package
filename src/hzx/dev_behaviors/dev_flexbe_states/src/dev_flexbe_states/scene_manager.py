#!/usr/bin/env python
import sys
import rospy
import rostopic
import moveit_commander
import geometry_msgs.msg
from flexbe_core import EventState, Logger

'''
Updated on Apr 22st, 2024
@author: zixuan.huang@leapting.com
'''


class SceneManager(EventState):
    '''
    A state that can attach a object to the robot or detach the object from the robot.

    -- action               string          "attach" or "detach".
    -- object_size          list            [x, y, z], unit is meter
    -- frame_id             string          reference_frame of box
    -- box_name             string          the name of the object to attach
    -- box_position         list            The position between the box and the frame_id, unit is meter

    <= done		    action complete.
    '''

    def __init__(self, action="attach",
                 object_size=[2.278, 1.134, 0.035],
                 frame_id="tool0",
                 box_name="pvm",
                 box_position=[0, 0, 0]):
        super(SceneManager, self).__init__(outcomes=['done'])
        moveit_commander.roscpp_initialize(sys.argv)
        self._action = action
        self._size = object_size
        self._position = box_position
        self.box_name = box_name
        self.eef_link = frame_id
        self.box_pose = geometry_msgs.msg.PoseStamped()
        
    def execute(self, userdata):

        msg_type, msg_topic, _ = rostopic.get_topic_class('/move_group/goal')
        if msg_topic is None:
            Logger.logwarn('Moveit Failed')
            return
        else:
            self.scene = moveit_commander.PlanningSceneInterface()

        if self._action == "attach":
            self.scene.add_box(self.box_name, self.box_pose, size=(self._size[0],self._size[1], self._size[2]))
            if not (self.wait_for_state_update(box_is_known=True) or 
                    self.wait_for_state_update(box_is_known=True,box_is_attached=True)):
                return      
            self.scene.attach_box(self.eef_link, self.box_name, touch_links=[])
            if not self.wait_for_state_update(box_is_attached=True):
                return
            Logger.loginfo('SceneManager attach succeed: executed')
            return 'done'
            
        else:
            self.scene.remove_attached_object(self.eef_link, name=self.box_name)
            if not (self.wait_for_state_update(box_is_known=True) or 
                    self.wait_for_state_update()):
                return
            self.scene.remove_world_object(self.box_name)
            if not self.wait_for_state_update():
                return
            Logger.loginfo('SceneManager detach succeed: executed')
            return 'done'


    def on_enter(self, userdata):
        if self.box_name == "pvm":
            if rospy.has_param('robot_state/pvm_length'):
                self._size[0] = rospy.get_param('robot_state/pvm_length') * 0.001
            if rospy.has_param('robot_state/pvm_width'):
                self._size[1] = rospy.get_param('robot_state/pvm_width') * 0.001
        self.box_pose.header.frame_id = self.eef_link
        self.box_pose.pose.position.x = self._position[0] 
        self.box_pose.pose.position.y = self._position[1]
        self.box_pose.pose.position.z = self._position[2]

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False):
        attached_objects = self.scene.get_attached_objects([self.box_name])
        is_attached = len(attached_objects.keys()) > 0
        is_known = self.box_name in self.scene.get_known_object_names()
        if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True
        else:
            return False
