#!/usr/bin/env python
from flexbe_core import EventState, Logger
import rospy
import moveit_commander
import sys

'''
@author: lei.zeng@leapting.com
'''


class ArmPoseCheck(EventState):
    def __init__(self,
                 x_max=None,
                 x_min=None,
                 y_max=None,
                 y_min=None,
                 z_max=None,
                 z_min=None):
        super(ArmPoseCheck, self).__init__(outcomes=['done', 'failed'],
                                           input_keys=['move_group'])
        self.x_max = x_max
        self.x_min = x_min
        self.y_max = y_max
        self.y_min = y_min
        self.z_max = z_max
        self.z_min = z_min
        self.res = 'failed'

    def manipulator_init(self):
        group_name = rospy.get_param(
            "/move_group/moveit_sim_hw_interface/joint_model_group", "leapting_arm")
        moveit_commander.roscpp_initialize(sys.argv)
        self._move_group = moveit_commander.MoveGroupCommander(group_name)
        Logger.loginfo('%s:self manipulation init.' % self.name)

    def on_enter(self, userdata):
        if userdata.move_group != None:
            self._move_group = userdata.move_group
            Logger.loginfo('%s:manipulation init by userdata.' % self.name)
        else:
            self.manipulator_init()

        current_pose = self._move_group.get_current_pose().pose
        if (
            self.if_larger(current_pose.position.x, self.x_max)
            and self.if_smaller(current_pose.position.x, self.x_min)
            and self.if_larger(current_pose.position.y, self.y_max)
            and self.if_smaller(current_pose.position.y, self.y_min)
            and self.if_larger(current_pose.position.z, self.z_max)
            and self.if_smaller(current_pose.position.z, self.z_min)
        ):
            self.res = 'done'
        else:
            self.res = 'failed'

    def execute(self, userdata):
        return self.res

    def if_larger(self, v, v_max):
        print(v, v_max, v_max == None)

        if v_max == None:
            return True
        else:
            return (v <= v_max)

    def if_smaller(self, v, v_min):
        print(v, v_min)
        if v_min == None:
            return True
        else:
            return (v >= v_min)
