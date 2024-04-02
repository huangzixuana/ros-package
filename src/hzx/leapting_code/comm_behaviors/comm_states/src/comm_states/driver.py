#!/usr/bin/env python

import json
import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String

class driver(EventState):
    '''
    Publish String & Subscribe String/DiagnosticArray topic

    -- topic            stirng              topic name, as publisher
    -- name             string              event name, as subscriber, trigger driver's event list
    -- mb_addr          string/int/float    mb address
    -- mb_data          string/int/float    mb data
    -- timeoff          float               time to repeat writing
    -- timeout          float               time to reading failed

    <= done             publishing done
    <= failed           publishing failed

    '''

    def __init__(self, topic='', name='', mb_addr='', mb_data='', timeoff=1.0, timeout=0.0):
        super(driver, self).__init__(outcomes=['done', 'failed'], input_keys=['topic', 'name', 'mb_addr', 'mb_data'], output_keys=['mb_expect', 'mb_except'])
        self._update_pub = ProxyPublisher({'flexbe/behavior_updating': String})

        self._topic = topic
        self._mb = {}
        self._mb['name'] = name
        self._mb['mb_addr'] = mb_addr
        self._mb['mb_data'] = mb_data
        self._timeoff = timeoff
        self._timeout = timeout

        if self._topic != '' and self._topic != 'null':
            self._pub = ProxyPublisher({self._topic: String})
        if self._mb['name'] != '' and self._mb['name'] != 'null':
            if self._mb['name'].rsplit('_', 1)[1] == 'response':
                self._type = 'String'
                self._sub = ProxySubscriberCached({self._mb['name']: String})
            else:
                self._type = 'DiagnosticArray'
                self._sub = ProxySubscriberCached({self._mb['name']: DiagnosticArray})

        self._time_publish = rospy.Time.now()
        self._time_enter = self._time_publish
        self._time_execute = self._time_publish

    def execute(self, userdata):
        '''main loop'''
        self._time_execute = rospy.Time.now()

        if self._timeoff <= 0.0 or self._mb['name'] == '' or self._mb['name'] == 'null':
            return 'done'
        if self._timeout > 0.0:
            if (self._time_execute - self._time_enter) >= rospy.Duration(self._timeout):
                return 'failed'
        if self._topic != '' and self._topic != 'null':
            if (self._time_execute - self._time_publish) >= rospy.Duration(self._timeoff):
                msg = String()
                msg.data = json.dumps(self._mb)
                self._pub.publish(self._topic, msg)
                self._time_publish = rospy.Time.now()

        if self._sub.has_msg(self._mb['name']):
            msg = self._sub.get_last_msg(self._mb['name'])
            self._sub.remove_last_msg(self._mb['name'])
            if self._type == 'String':
                userdata.mb_except = json.loads(msg.data)['mb_addr']
                userdata.mb_expect = json.loads(msg.data)['mb_data']
                if self._mb['mb_addr'] != '' and \
                    self.is_equal(self._mb['mb_addr'], userdata.mb_except):
                    return 'failed'
                if self._mb['mb_data'] == '' or \
                    self.is_equal(self._mb['mb_data'], userdata.mb_expect):
                    return 'done'
            elif self._type == 'DiagnosticArray':
                for i in range(len(msg.status)):
                    # if self._mb['name'] == '' or self._mb['name'] == msg.status[i].hardware_id:
                    for j in range(len(msg.status[i].values)):
                        if msg.status[i].values[j].key == 'mb_except':
                            userdata.mb_except = float(msg.status[i].values[j].value)
                        if msg.status[i].values[j].key == 'mb_expect':
                            userdata.mb_expect = float(msg.status[i].values[j].value)
                if userdata.mb_except > 0:
                    return 'failed'
                elif self.is_equal(self._mb['mb_data'], userdata.mb_expect):
                    return 'done'

    def is_equal(self, data0, data1):
        '''judge'''
        if isinstance(data0, float) or isinstance(data0, int):
            if abs(float(data1) - data0) < 0.0001:
                return True
        elif isinstance(data0, str):
            if str(data1) == data0:
                return True
            elif data0[0:1] == "=":
                if float(data1) == float(data0[1:]):
                    return True
            elif len(data0) > 1 and (data0[0:2] == "~=" or data0[0:2] == "=~"):
                factor = 1
                if len(data0.split('.')) > 1:
                    factor = 10**len(data0.split('.')[1])
                if round(float(data1) * factor) == round(float(data0[2:]) * factor):
                    return True
            elif len(data0) > 1 and (data0[0:2] == "!=" or data0[0:2] == "=!"):
                if float(data1) != float(data0[2:]):
                    return True
            elif len(data0) > 1 and (data0[0:2] == ">=" or data0[0:2] == "=>"):
                if float(data1) >= float(data0[2:]):
                    return True
            elif len(data0) > 1 and (data0[0:2] == "<=" or data0[0:2] == "=<"):
                if float(data1) <= float(data0[2:]):
                    return True
            elif data0[0:1] == ">":
                if float(data1) > float(data0[1:]):
                    return True
            elif data0[0:1] == "<":
                if float(data1) < float(data0[1:]):
                    return True
            else:
                factor = 1
                if len(data0.split('.')) > 1:
                    factor = 10**len(data0.split('.')[1])
                if round(float(data1) * factor) == round(float(data0) * factor):
                    return True
        return False

    def on_enter(self, userdata):
        '''subscribe'''
        self._time_enter = rospy.Time.now()

        update_msg = String()
        update_msg.data = self.name
        self._update_pub.publish('flexbe/behavior_updating', update_msg)

        if self._topic == '':
            self._topic = userdata.topic
            if self._topic != '' and self._topic != 'null':
                self._pub = ProxyPublisher({self._topic: String})
        if self._mb['name'] == '':
            self._mb['name'] = userdata.name
            if self._mb['name'] != '' and self._mb['name'] != 'null':
                if self._mb['name'].rsplit('_', 1)[1] == 'response':
                    self._type = 'String'
                    self._sub = ProxySubscriberCached({self._mb['name']: String})
                else:
                    self._type = 'DiagnosticArray'
                    self._sub = ProxySubscriberCached({self._mb['name']: DiagnosticArray})
        if self._mb['mb_addr'] == '':
            self._mb['mb_addr'] = userdata.mb_addr
        if self._mb['mb_data'] == '':
            self._mb['mb_data'] = userdata.mb_data

        if self._topic != '' and self._topic != 'null':
            msg = String()
            msg.data = json.dumps(self._mb)
            self._pub.publish(self._topic, msg)
            self._time_publish = rospy.Time.now()
        if self._mb['name'] != '' and self._mb['name'] != 'null':
            self._sub.enable_buffer(self._mb['name'])
            if self._sub.has_msg(self._mb['name']):
                self._sub.remove_last_msg(self._mb['name'])

    def on_exit(self, userdata):
        '''unsubscribe'''
        if self._mb['name'] != '' and self._mb['name'] != 'null':
            self._sub.disable_buffer(self._mb['name'])
