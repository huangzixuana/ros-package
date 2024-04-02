#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import Header, String
import rospy

'''
Created on 06.01.2023
@author: corey.wang
'''


class PublishHeaderState(EventState):
    '''
    Publish a message to /task_switch

    -- seq	         int       seq
    -- frame_id      string    frame_id
># twist		Twist			Velocity command to be published.
    <= done              publishing done

    '''

    def __init__(self, seq, frame_id):
        '''
        Constructor
        '''
        super(PublishHeaderState, self).__init__(outcomes=['done'],	input_keys=['header'])
        self._update_pub = ProxyPublisher({'flexbe/behavior_updating': String})
        self._seq = seq
        self._frame_id = frame_id
        self._topic = 'trig'
        self._pub = ProxyPublisher({self._topic: Header})

    def execute(self, userdata):
        return 'done'

    def on_enter(self, userdata):
        update_msg = String()
        update_msg.data = self.name
        self._update_pub.publish('flexbe/behavior_updating', update_msg)

        msg = Header()
        msg.seq = self._seq
        msg.frame_id = self._frame_id
        msg.stamp = rospy.Time.now()
        self._pub.publish(self._topic, userdata.header)
