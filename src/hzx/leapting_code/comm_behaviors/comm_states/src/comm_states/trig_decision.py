# !/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from std_msgs.msg import Header
import time


'''
Created on 2023-11-24
@author: lei.zeng@leapting.com
'''


class TrigDecision(EventState):
    '''
    judge trig

    <=  next     string  next
    <=  withdraw       string  withdraw
    '''

    def __init__(self):
        super(TrigDecision, self).__init__(outcomes=['next', 'withdraw'])

        self._trig_sub = ProxySubscriberCached(
            {'trig': Header})

        self._res = None
        self._received = False

    def trig_cb(self, msg):
        if msg.frame_id == "next" or msg.frame_id == "withdraw":
            self._res = msg.frame_id
            self._received = True

    def on_enter(self, userdata):
        self._trig_sub.subscribe('trig', Header,
                                 callback=self.trig_cb, buffered=False)
        self._received = False

    def execute(self, userdata):
        if self._received:
            return self._res
        
    def on_exit(self, userdata):
        self._trig_sub.unsubscribe_topic('trig')
