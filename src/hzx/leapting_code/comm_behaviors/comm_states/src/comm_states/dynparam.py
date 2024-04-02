#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
import dynamic_reconfigure.client
from std_msgs.msg import String

class dynparam(EventState):
    '''
    Set dynamic parameter (most type value)

    -- client	   string	   	   	   	   	   client name
    -- parameter   string	   	   	   	   	   parameter
    -- value       string/bool/int/float	   value

    <= done                   	   	   	   	   setting done

    '''

    def __init__(self, client='', parameter='', value=''):
        super(dynparam, self).__init__(outcomes=['done'],
                                       input_keys=['client', 'parameter', 'value'],
                                       output_keys=['client0', 'parameter0', 'value0'])
        self._update_pub = ProxyPublisher({'flexbe/behavior_updating': String})
        self._client = client
        self._parameter = parameter
        self._value = value

    def execute(self, userdata):
        return 'done'

    def on_enter(self, userdata):
        update_msg = String()
        update_msg.data = self.name
        self._update_pub.publish('flexbe/behavior_updating', update_msg)

        if self._client == '':
            self._client = userdata.client
        if self._parameter == '':
            self._parameter = userdata.parameter
        if self._value == '':
            self._value = userdata.value

        client_recfg = dynamic_reconfigure.client.Client(self._client)
        params0 = client_recfg.get_configuration()
        userdata.client0 = self._client
        userdata.parameter0 = self._parameter
        userdata.value0 = params0[userdata.parameter0]
        params = {self._parameter: self._value}
        client_recfg.update_configuration(params)

        # TODO:
    # def on_exit(self, userdata):
    #     self.cancel_active_goals()
