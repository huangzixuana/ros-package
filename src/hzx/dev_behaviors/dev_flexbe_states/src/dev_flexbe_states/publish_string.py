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
        self._pub.publish(self._topic, t)
