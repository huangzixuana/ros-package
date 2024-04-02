#!/usr/bin/env python

import json
import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached
from std_msgs.msg import String

class browser(EventState):
    '''
    Publish String & Subscribe String/DiagnosticArray topic

    -- robot_topic      string      pub robot_topic
    -- robot_data       string      pub robot_data
    -- browser_topic    string      sub browser_topic
    -- browser_data     string      sub browser_data

    <= done             publishing done
    <= timeout           publishing timeout
    '''

    def __init__(self, robot_topic='robot_msg', robot_data='', browser_topic='browser_msg', browser_data='', timeoff=1.0, timeout=0.0):
        super(browser, self).__init__(outcomes=['done', 'timeout'])
        self._update_pub = ProxyPublisher({'flexbe/behavior_updating': String})

        self._robot_topic = robot_topic
        self._robot_data = robot_data
        self._browser_topic = browser_topic
        self._browser_data = browser_data
        self._timeoff = timeoff
        self._timeout = timeout

        if self._robot_topic != '':
            self._pub = ProxyPublisher({self._robot_topic: String})
        if self._browser_topic != '':
            self._sub = ProxySubscriberCached({self._browser_topic: String})

        self._time_publish = rospy.Time.now()
        self._time_enter = self._time_publish
        self._time_execute = self._time_publish

    def execute(self, userdata):
        '''main loop'''
        self._time_execute = rospy.Time.now()

        if self._timeoff <= 0.0 or self._browser_topic == '':
            return 'done'
        if self._timeout > 0.0:
            if (self._time_execute - self._time_enter) >= rospy.Duration(self._timeout):
                return 'timeout'
        if self._robot_topic != '':
            if (self._time_execute - self._time_publish) >= rospy.Duration(self._timeoff):
                msg = String()
                msg.data = self._robot_data
                self._pub.publish(self._robot_topic, msg)
                self._time_publish = rospy.Time.now()

        if self._sub.has_msg(self._browser_topic):
            msg = self._sub.get_last_msg(self._browser_topic)
            self._sub.remove_last_msg(self._browser_topic)
            if self._browser_data == msg.data:
                return 'done'

    def on_enter(self, userdata):
        '''subscribe'''
        self._time_enter = rospy.Time.now()

        update_msg = String()
        update_msg.data = self.name
        self._update_pub.publish('flexbe/behavior_updating', update_msg)

        if self._browser_topic != '':
            self._sub.enable_buffer(self._browser_topic)
            if self._sub.has_msg(self._browser_topic):
                self._sub.remove_last_msg(self._browser_topic)

        if self._robot_topic != '':
            msg = String()
            msg.data = self._robot_data
            self._pub.publish(self._robot_topic, msg)
            self._time_publish = rospy.Time.now()

    def on_exit(self, userdata):
        '''unsubscribe'''
        if self._browser_topic != '':
            self._sub.disable_buffer(self._browser_topic)
