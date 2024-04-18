#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import Header, String
from sensor_msgs.msg import JoyFeedback, JoyFeedbackArray
import rospy

class joy_feedback_array_pub(EventState):
    '''
    Publish JoyFeedbackArray topic

    -- topic        	stirng	topic name
    -- data             string	event name, trigger driver's event list

    <= done             publishing done

    '''

    def __init__(self, topic, data_type, data_id, data_intensity):
        super(joy_feedback_array_pub, self).__init__(outcomes=['done'], input_keys=['topic'])
        self._update_pub = ProxyPublisher({'flexbe/behavior_updating': String})

        self._data = JoyFeedback()
        self._data.type = data_type
        self._data.id = data_id
        self._data.intensity = data_intensity
        self._topic = topic
        self._pub = ProxyPublisher({self._topic: JoyFeedbackArray})

    def execute(self, userdata):
        return 'done'

    def on_enter(self, userdata):
        update_msg = String()
        update_msg.data = self.name
        self._update_pub.publish('flexbe/behavior_updating', update_msg)

        if self._topic == '':
            self._topic = userdata.topic
        self._pub = ProxyPublisher({self._topic: String})

        msg = JoyFeedbackArray()
        msg.array.append(self._data)
        self._pub.publish(self._topic, msg)