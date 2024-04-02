#!/usr/bin/env python

# import roslaunch
import rospy
import time
import sys
import math
from math import radians

import os


import git
import json
from std_msgs.msg import Header, String
from sensor_msgs.msg import Joy
import numpy


def main(argv):

    rospy.init_node('test_node')
    pub = rospy.Publisher(
        'robot_command', String, queue_size=10)

    # d = {'parameter': {'pvm_length': 2500, 'pvm_width': 1234}, 'dynparam': {'cmd_vel_filter': {'filter_enabled': None, 'test_p': None}, 'test': {
    #     'dyp1': None, 'dyp2': None}}, 'flexbe': {'beh1': {'param1': 1, 'param2': 'string2', 'param3': False}, 'beh2': {'param1': 3, 'param5': -4.5}}}

    d = {'git': {'op': 'pull'}}
    msg = String()
    msg.data = json.dumps(d, sort_keys=True)
    data = json.loads(msg.data)

    time.sleep(3)
    pub.publish(msg)


if __name__ == '__main__':
    main(sys.argv)
