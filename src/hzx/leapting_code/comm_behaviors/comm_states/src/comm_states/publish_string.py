#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import String
import rospy

'''
Created on 11.04.2024
'''

class PublishString(EventState):
    '''
    Publishes a string message.
    -- name     string		The topic name.
    -- value    string      The topic value.
    >= value 					Value of string.
    <= done 					Done publishing.
    '''

    def __init__(self, name="", value=""):
        super(PublishString, self).__init__(outcomes=['done'])
        self._topic = name
        self._value = value
        self._pub = ProxyPublisher({self._topic: String})

    def execute(self, userdata):
        return 'done'

    def on_enter(self, userdata):
        t = String()
        t.data = self._value
        if self._value.__contains__('{'):
            dic = eval(self._value)
            if "lift_goal_height" in dic.keys():
                scale_value = int(10000 * dic["lift_goal_height"])
                # dic["lift_goal_height"] = scale_value
                t.data = self._value.replace(str(dic["lift_goal_height"]),str(scale_value))
            elif "rotate_goal_angle" in dic.keys():
                scale_value = int(10 * dic["rotate_goal_angle"])
                # dic["rotate_goal_angle"] = scale_value
                t.data = self._value.replace(str(dic["rotate_goal_angle"]),str(scale_value))
        self._pub.publish(self._topic, t)
