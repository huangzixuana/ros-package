#!/usr/bin/env python
from flexbe_core import EventState, Logger
import tf
from tf import TransformListener
import time
import rospy
import math

'''
Created on Jun 2nd, 2023
@author: lei.zeng@leapting.com, chenxin.zou@leapting.com
'''


class ListenState(EventState):
    '''
    check if the Transformer can determine the transform from source_frame to target_frame within given time.

    -- target_frame	       string            target frame to check
    -- source_frame        string            parent frame
    -- timeout             float             timeout
    -- fresh_time          float             tf fresh time
    -- ideal               list              six ideal degrees
    -- tolerance           list              tolerance for each degree


    <= done                  transform determined
    <= timeout               timeout
    '''

    def __init__(self, target_frame='', source_frame='tool0', timeout=60, fresh_time=3.0,
                 ideal=[0, 0, 0, 0, 0, 0], tolerance=[-1, -1, -1, -1, -1, -1]):
        super(ListenState, self).__init__(outcomes=['done', 'timeout'])
        self._target_frame = target_frame
        self._source_frame = source_frame
        self._timeout = timeout
        self._fresh_time = fresh_time

        self._ideal = ideal
        self._tolerance = tolerance

        self.tf1_lis = TransformListener()

    def on_enter(self, userdata):
        self.tf1_lis.clear()
        self.start_time = time.time()

    def execute(self, userdata):
        if ((time.time() - self.start_time) < self._timeout):
            # if self.tf1_lis.frameExists(self._target_frame):
            try:
                t = self.tf1_lis.getLatestCommonTime(
                    self._source_frame,  self._target_frame)
                p, q = self.tf1_lis.lookupTransform(
                    self._source_frame, self._target_frame, t)

                if abs((rospy.Time.now()-t).to_sec()) > self._fresh_time:
                    Logger.logwarn('%s: tf not fresh' % self.name)
                    Logger.loginfo('Now t: %.2f', rospy.Time.now().to_sec())
                    Logger.loginfo('TF t: %.2f', t.to_sec())
                    return

                # judge whether six degrees is ideal with some tolerance
                judge_index = []
                for i in range(len(self._tolerance)):
                    if self._tolerance[i] != -1:
                        judge_index.append(i)
                if len(judge_index) != 0:
                    eul = tf.transformations.euler_from_quaternion(q)
                    p_eul = [p[0], p[1], p[2], eul[0], eul[1], eul[2]]
                    Logger.loginfo('Real: parent is %s, child is %s, P:({0:.3f}, {1:.3f}, {2:.3f}), RPY: ({3:.3f}, {4:.3f}, {5:.3f})'.format(
                        p[0], p[1], p[2], eul[0], eul[1], eul[2]) % (self._source_frame, self._target_frame))

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

                return 'done'
            except Exception as e:
                Logger.logwarn('%s: exception tf1: %s' %
                               (self.name, str(e)))
        else:
            return 'timeout'
