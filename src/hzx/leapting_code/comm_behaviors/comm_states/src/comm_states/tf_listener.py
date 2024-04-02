#!/usr/bin/env python

from flexbe_core import EventState, Logger
import rospy
import tf
import time

'''
Updated on Jun 26, 2023
@author: lei.zeng@tu-dortmund.de
'''


class TFListener(EventState):
    '''
    check target tf existence
    -- timeout	         float            time to check
    -- frame_id          string       frame id as reference

    <= done                  find the target TF withinallotted time
    <= timeout               timeout
    '''

    def __init__(self, timeout=20, frame_id="map"):
        super(TFListener, self).__init__(outcomes=['done', 'timeout'])
        self._timeout = timeout
        self._frame_id = frame_id
        self._res = 'timeout'
      
        self._tf_listener = tf.TransformListener()

    def on_enter(self, userdata):
        self._res = 'timeout'
        t_s = time.time()
        while time.time()-t_s < self._timeout and self._res == 'timeout':
            if self._tf_listener.frameExists(self._frame_id):
                self._res = 'done'
            else:
                pass
            time.sleep(0.5)

    def execute(self, userdata):
        return self._res
