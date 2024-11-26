#!/usr/bin/env python

from flexbe_core import EventState
import subprocess
'''
Created on 2020-09-28
@author: lei.zeng@tu-dortmund.de
'''


class SubProcess(EventState):
    '''
    subprocess exe
    -- cmd	         string       cmd

    <= done              publishing done
    '''

    def __init__(self, cmd):
        super(SubProcess, self).__init__(outcomes=['done'])
        self._cmd = cmd

    def on_enter(self, userdata):
        subprocess.Popen(self._cmd, shell=True)

    def execute(self, userdata):
        return 'done'
