#!/usr/bin/env python

from flexbe_core import EventState, Logger
import rospy
import sys
import moveit_commander


'''
Updated on Sep 19th, 2023
@author: lei.zeng@leapting.com
'''


class ManipulationShare(EventState):
    def __init__(self, reference_frame="base_arm",
                 end_effector_link="tool0"):
        super(ManipulationShare, self).__init__(outcomes=['done'],
                                                output_keys=['move_group'])
        self._reference_frame, self._end_effector_link = reference_frame, end_effector_link
        self._init_done = False
        rospy.Timer(rospy.Duration(0.1), self.time_cb, oneshot=True)

    def time_cb(self, event):
        self.manipulator_init()

    def manipulator_init(self):
        group_name = rospy.get_param(
            "/move_group/moveit_sim_hw_interface/joint_model_group", "leapting_arm")
        moveit_commander.roscpp_initialize(sys.argv)
        self._move_group = moveit_commander.MoveGroupCommander(group_name)
        if self._reference_frame != "":
            self._move_group.set_pose_reference_frame(self._reference_frame)
        if self._end_effector_link != "":
            self._move_group.set_end_effector_link(self._end_effector_link)
        self._init_done = True
        Logger.loginfo('%s:self manipulation init.' % self.name)

    def on_enter(self, userdata):
        pass

    def execute(self, userdata):
        if self._init_done:
            userdata.move_group = self._move_group
            return 'done'
