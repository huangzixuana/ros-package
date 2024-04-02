#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import String
import webbrowser


'''
Created on 2021-6-28
Update on 2022-01-06
@author: lei.zeng@tu-dortmund.de
'''


class WebBrowser(EventState):
    '''
    web browser
    -- web   string          web address

    <= done          

    '''

    def __init__(self, web="http://localhost/#"):
        super(WebBrowser, self).__init__(outcomes=['done'],
                                         input_keys=['open_web'])
        self._web = web
        self._update_pub = ProxyPublisher({'flexbe/behavior_updating': String})

    def on_enter(self, userdata):
        update_msg = String()
        update_msg.data = self.name
        self._update_pub.publish('flexbe/behavior_updating', update_msg)

    def execute(self, userdata):
        if userdata.open_web:
            if webbrowser.open(self._web):
                return 'done'
            else:
                pass
        else:
            return 'done'
