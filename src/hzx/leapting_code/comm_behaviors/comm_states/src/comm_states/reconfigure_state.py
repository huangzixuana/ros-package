#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
import dynamic_reconfigure.client
from std_msgs.msg import Header, String
import rospy
'''
Created on 09.12.2019
@author: lei.zeng@tu-dortmund.de
'''


class ReconfigureState(EventState):
    '''
    Set dynamic paramter (most type value)

    -- client	   string	   	   	   	   	   client name
    -- parameter   string	   	   	   	   	   parameter
    -- value       string/bool/int/float	   value

    <= done                   	   	   	   	   setting done

    '''

    def __init__(self, client, parameter, value):
        '''
        Constructor
        '''
        super(ReconfigureState, self).__init__(outcomes=['done'])
        self._update_pub = ProxyPublisher({'flexbe/behavior_updating': String})
        self._client = client
        self._paramter = parameter
        self._value = value

    def execute(self, userdata):
        try:
            param_value = rospy.get_param(self._client+'/'+self._paramter)
            if rospy.get_param(self._client+'/'+self._paramter) == self._value:
                return 'done'
            else:
                self.set_param()
        except Exception as e:
            Logger.logwarn('%s: %s' % (self.name, str(e)))

    def on_enter(self, userdata):
        update_msg = String()
        update_msg.data = self.name
        self._update_pub.publish('flexbe/behavior_updating', update_msg)

        if isinstance(self._value, str) and (self._value[0] == '='):
            try:
                self._value = str(rospy.get_param(self._value[1:]))
            except Exception as e:
                Logger.logwarn('%s: %s' % (self.name, str(e)))

        self.set_param()

    def set_param(self):
        try:
            client_recfg = dynamic_reconfigure.client.Client(
                self._client, timeout=10)
            params = {self._paramter: self._value}
            client_recfg.update_configuration(params)
        except Exception as e:
            Logger.logwarn('%s: %s' % (self.name, str(e)))
