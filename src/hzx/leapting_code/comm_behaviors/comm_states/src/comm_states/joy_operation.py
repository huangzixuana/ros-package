# !/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from sensor_msgs.msg import JoyFeedback, JoyFeedbackArray
from std_msgs.msg import String

'''
Created on 15.05.2020
@author: lei.zeng@tu-dortmund.de
'''


class JoyOperation(EventState):
    '''
    publish autocharge info to topic

    <= done  published
    '''

    def __init__(self, topic, idd, intensity, typee):
        super(JoyOperation, self).__init__(outcomes=['done'])
        self._topic = topic
        self._update_pub = ProxyPublisher({'flexbe/behavior_updating': String})
        self._joy_start_pub = ProxyPublisher({self._topic: JoyFeedbackArray})
        self._id = idd
        self._intensity = intensity
        self._type = typee

    def on_enter(self, userdata):
        update_msg = String()
        update_msg.data = self.name
        self._update_pub.publish('flexbe/behavior_updating', update_msg)

        joy_feedback_msg = JoyFeedbackArray()
        discharge_msg = JoyFeedback()
        joy_feedback_msg.array.append(discharge_msg)
        joy_feedback_msg.array[0].id = self._id
        joy_feedback_msg.array[0].intensity = self._intensity
        joy_feedback_msg.array[0].type = self._type
        self._joy_start_pub.publish(self._topic, joy_feedback_msg)

    def execute(self, userdata):
        return 'done'
