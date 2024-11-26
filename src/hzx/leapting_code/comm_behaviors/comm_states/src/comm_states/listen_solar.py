#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
import time
import rospy
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf

'''
Created on 2023-11-24
@author: lei.zeng@leapting.com, chenxin.zou@leapting.com
'''


class ListenSolar(EventState):
    '''
    check PoseWithCovarianceStamped within given time.

    -- timeout             float             timeout
    -- fresh_time          float             tf fresh time
    -- ideal               list              six ideal degrees
    -- tolerance           list              tolerance for each degree
    -- pose_topic          string            PoseWithCovarianceStamped topic name


    <= done                  transform determined
    <= timeout               timeout
    '''

    def __init__(self, timeout=60, fresh_time=3.0,
                 ideal=[0, 0, 0, 0, 0, 0], tolerance=[-1, -1, -1, -1, -1, -1],
                 pose_topic="filter_solar_pose"):
        super(ListenSolar, self).__init__(
            outcomes=['done', 'timeout'],  output_keys=['pose_msg'])

        self._pose_sub = ProxySubscriberCached(
            {pose_topic: PoseWithCovarianceStamped})
        self._pose_sub.subscribe(pose_topic, PoseWithCovarianceStamped,
                                 callback=self.solar_cb, buffered=False)
        self._received = False
        self._solar_msg = None

        self._timeout = timeout
        self._fresh_time = fresh_time

        self._ideal = ideal
        self._tolerance = tolerance

    def solar_cb(self, msg):
        self._solar_msg = msg
        self._received = True

    def on_enter(self, userdata):
        self.start_time = time.time()
        self._received = False

    def execute(self, userdata):
        if ((time.time() - self.start_time) < self._timeout):
            try:
                if self._received:
                    p = [self._solar_msg.pose.pose.position.x,
                         self._solar_msg.pose.pose.position.y,
                         self._solar_msg.pose.pose.position.z]

                    q = [self._solar_msg.pose.pose.orientation.x,
                         self._solar_msg.pose.pose.orientation.y,
                         self._solar_msg.pose.pose.orientation.z,
                         self._solar_msg.pose.pose.orientation.w]
                else:
                    return

                # judge whether six degrees is ideal with some tolerance
                judge_index = []
                for i in range(len(self._tolerance)):
                    if self._tolerance[i] != -1:
                        judge_index.append(i)
                if len(judge_index) != 0:
                    eul = tf.transformations.euler_from_quaternion(q)
                    p_eul = [p[0], p[1], p[2], eul[0], eul[1], eul[2]]
                    Logger.loginfo('Real: P:({0:.3f}, {1:.3f}, {2:.3f}), RPY: ({3:.3f}, {4:.3f}, {5:.3f})'.format(
                        p[0], p[1], p[2], eul[0], eul[1], eul[2]))

                    six_name = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
                    ok_index = []
                    bad_index = []
                    for index in judge_index:
                        min_temp = [self._ideal[index] -
                                    abs(self._tolerance[index])]
                        max_temp = [self._ideal[index] +
                                    abs(self._tolerance[index])]
                        if index in [3, 4, 5]:
                            if min_temp[0] < -math.pi and max_temp[0] < math.pi:
                                min2 = 2*math.pi + min_temp[0]
                                min_temp[0] = -math.pi
                                min_temp.append(min2)
                                max_temp.append(math.pi)
                            elif max_temp[0] > math.pi and min_temp[0] > -math.pi:
                                max2 = -2*math.pi + max_temp[0]
                                max_temp[0] = math.pi
                                max_temp.append(max2)
                                min_temp.append(-math.pi)
                            elif max_temp[0] > math.pi and min_temp[0] < -math.pi:
                                min_temp[0] = -math.pi
                                max_temp[0] = math.pi
                        if len(min_temp) == 1:
                            Logger.loginfo('Judge: %s in [{0},{1}]'.format(
                                min_temp[0], max_temp[0]) % (six_name[index]))
                            if p_eul[index] >= min_temp[0] and p_eul[index] <= max_temp[0]:
                                ok_index.append(index)
                            else:
                                bad_index.append(index)
                        else:
                            Logger.loginfo('Judge: %s in [{0},{1}] or [{2},{3}]'.format(
                                min_temp[0], max_temp[0], min_temp[1], max_temp[1]) % (six_name[index]))
                            if (p_eul[index] >= min_temp[0] and p_eul[index] <= max_temp[0]) or (p_eul[index] >= min_temp[1] and p_eul[index] <= max_temp[1]):
                                ok_index.append(index)
                            else:
                                bad_index.append(index)
                    if len(ok_index) != 0:
                        ok_info = 'OK:'
                        for index in ok_index:
                            ok_info += ' '+str(six_name[index])+','
                        Logger.loginfo(ok_info[:-1])
                    if len(bad_index) != 0:
                        bad_info = 'Bad:'
                        for index in bad_index:
                            bad_info += ' '+str(six_name[index])+','
                        Logger.logwarn(bad_info[:-1])
                        return

                userdata.pose_msg = self._solar_msg
                return 'done'
            except Exception as e:
                Logger.logwarn('%s: exception : %s' %
                               (self.name, str(e)))
        else:
            return 'timeout'
