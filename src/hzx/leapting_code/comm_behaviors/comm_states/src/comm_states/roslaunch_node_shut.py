#! /usr/bin/env python
# -*- encoding:utf-8 -*-
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import Header, String
import rospy
import subprocess
import time as t


class roslaunch_node_shut(EventState):
    '''
    Publish a message to /task_switch
    -- cmd	 string
    -- pkg   string
    --launch string
    <= done	publishing	done
    '''

    def __init__(self):
        '''
        Constructor
        '''
        super(roslaunch_node_shut, self).__init__(
            outcomes=['done'], input_keys=['shutdown_class'])

    def execute(self, userdata):
        return 'done'

    def on_enter(self, userdata):
        a = userdata.shutdown_class
        a.terminate()
        a.wait()
