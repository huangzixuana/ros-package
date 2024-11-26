#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import Header
import rospy

'''
Created on 06.01.2023
@author: corey.wang
'''


class PublishHeader(EventState):
    '''
    Publish a message to /trig

    -- seq	         int       seq
    -- frame_id      string    frame_id
    <= done              publishing done

    '''

    def __init__(self, seq, frame_id):
        '''
        Constructor
        '''
        super(PublishHeader, self).__init__(outcomes=['done'])
        self._seq = seq
        self._frame_id = frame_id
        self._pub = ProxyPublisher({'trig': Header})

    def execute(self, userdata):
        return 'done'

    def on_enter(self, userdata):
        msg = Header()
        msg.seq = self._seq
        msg.frame_id = self._frame_id
        msg.stamp = rospy.Time.now()
        self._pub.publish('trig', msg)
