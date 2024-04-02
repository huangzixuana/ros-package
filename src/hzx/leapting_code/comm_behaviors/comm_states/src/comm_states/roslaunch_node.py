#! /usr/bin/env python
# -*- encoding:utf-8 -*-
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import Header, String
import rospy
import subprocess
import time as t


class roslaunch_node(EventState):
    '''
    Publish a message to /task_switch
    -- cmd	 string
    -- pkg   string
    --launch string
    <= done	publishing	done
    '''

    def __init__(self, cmd, pkg, launch_Node):
        '''
        Constructor
        '''
        super(roslaunch_node, self).__init__(
            outcomes=['done'], output_keys=['output_value', 'shutdown_class'])
        self.cmd = []
        self.cmd.append(cmd)
        self.cmd.append(pkg)
        self.cmd.append(launch_Node)

    def execute(self, userdata):

        return 'done'

    def on_enter(self, userdata):

        a = subprocess.Popen(self.cmd)
        userdata.shutdown_class = a
