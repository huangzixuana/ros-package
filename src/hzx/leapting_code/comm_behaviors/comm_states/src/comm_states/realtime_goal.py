#!/usr/bin/env python
from flexbe_core import EventState, Logger
import tf
from tf import TransformListener
import time
import rospy
import numpy as np
from flexbe_core.proxy import ProxySubscriberCached
from geometry_msgs.msg import PoseWithCovarianceStamped

'''
Created on Jun 5th, 2023
@author: lei.zeng@leapting.com, chenxin.zou@leapting.com
'''


class RealtimeGoal(EventState):
    '''
    Dynamically generate goal and convert reference frame

    -- position              list         position [x,y,z]
    -- orientation           list         orientation [x,y,z,w]
    -- frame_id              string       frame id as reference
    -- source_frame          string       frame id as reference
    -- gap_pvm_pvm           float        gap between two adjacent PVMs
    -- gap_pvm_chassis       float        gap between PVM and chassis
    -- chassis_width         float        width of chassis

    #> goal  dict    goal dict with key: position, orientation, (new)frame_id

    <= done                  done
    '''

    def __init__(self, position=[0, 0, 0], orientation=[0, 0, 0.7071, 0.7071], frame_id='', source_frame='map', if_back=False, gap_pvm_pvm=0.35, gap_pvm_chassis=0, chassis_width=2.2):
        super(RealtimeGoal, self).__init__(
            outcomes=['done'], output_keys=['goal'])
        # orientation=[0, 0, 0.7071, 0.7071]
        self._position, self._orientation = position, orientation

        self._frame_id = frame_id
        self._source_frame = source_frame
        self._if_back = if_back

        self._gap_pvm_pvm = gap_pvm_pvm
        self._gap_pvm_chassis = gap_pvm_chassis
        self._chassis_width = chassis_width

        self.tf1_lis = TransformListener()

        self._solar_msg = None
        self._received = False
        self._solar_sub = ProxySubscriberCached(
            {'filter_solar_pose': PoseWithCovarianceStamped})
        self._solar_sub.subscribe('filter_solar_pose', PoseWithCovarianceStamped,
                                  callback=self.solar_cb, buffered=False)

    def solar_cb(self, msg):
        self._solar_msg = msg
        self._received = True

    def on_enter(self, userdata):
        self._received = False
        self.tf1_lis.clear()

        if rospy.has_param('robot_state/pvm_width'):
            self._position[1] = rospy.get_param(
                '/robot_state/pvm_width')*0.001*1.5+self._gap_pvm_pvm

            if self._if_back:
                self._position[1] = (rospy.get_param(
                    '/robot_state/pvm_width')*0.001*0.5+self._gap_pvm_pvm) * (-1.0)
        if rospy.has_param('robot_state/pvm_length'):
            self._position[0] = (rospy.get_param('robot_state/pvm_length') *
                                 0.001*0.5 + self._gap_pvm_chassis + self._chassis_width*0.5)*(-1.0)

    def execute(self, userdata):
        try:
            if self._source_frame == "base_link" and self._frame_id == "solar_link":
                b_p, b_q = self.tf1_lis.lookupTransform(
                    "base_link",
                    "base_arm", rospy.Time(0))
                T0 = tf.TransformerROS().fromTranslationRotation(b_p, b_q)
                if self._received:
                    p = [self._solar_msg.pose.pose.position.x,
                         self._solar_msg.pose.pose.position.y,
                         self._solar_msg.pose.pose.position.z]
                    q = [self._solar_msg.pose.pose.orientation.x,
                         self._solar_msg.pose.pose.orientation.y,
                         self._solar_msg.pose.pose.orientation.z,
                         self._solar_msg.pose.pose.orientation.w]
                    T1 = tf.TransformerROS().fromTranslationRotation(p, q)
                    T1 = np.dot(T0, T1)
                else:
                    return
            else:
                t = self.tf1_lis.getLatestCommonTime(self._source_frame,
                                                     self._frame_id)
                p, q = self.tf1_lis.lookupTransform(
                    self._source_frame,
                    self._frame_id, t)
                T1 = tf.TransformerROS().fromTranslationRotation(p, q)

            T2 = tf.TransformerROS().fromTranslationRotation(
                self._position, self._orientation)
            T = np.dot(T1, T2)



            t1 = tf.transformations.translation_from_matrix(T1)
            q1 = tf.transformations.quaternion_from_matrix(T1)

            Logger.loginfo("T1 base_arm solar_link: P,Q:({0}, {1}, {2}), ({3}, {4}, {5}, {6})"
                           .format(p[0], p[1], p[2], q[0], q[1], q[2], q[3]))
            Logger.loginfo("T1 base_link solar_link: P,Q:({0}, {1}, {2}), ({3}, {4}, {5}, {6})"
                           .format(t1[0], t1[1], t1[2], q1[0], q1[1], q1[2], q1[3]))
            Logger.loginfo("T2 solar_link target: P,Q:({0}, {1}, {2}), ({3}, {4}, {5}, {6})"
                           .format(self._position[0], self._position[1], self._position[2], self._orientation[0], self._orientation[1], self._orientation[2], self._orientation[3]))

            trans_t = list(tf.transformations.translation_from_matrix(T))
            eul = list(tf.transformations.euler_from_matrix(T))

            # before limitation
            Logger.loginfo("before limitation: nav P,Eul:({0}, {1}, {2}), ({3}, {4}, {5})"
                           .format(trans_t[0], trans_t[1], trans_t[2], eul[0], eul[1], eul[2]))

            trans_t[2] = 0
            # 分别对x和y进行限制
            if self._if_back:
                trans_t[0] = np.clip(trans_t[0], -2, 0)
            else:
                trans_t[0] = np.clip(trans_t[0], 0, 3)

            trans_t[1] = np.clip(
                trans_t[1], -0.15*abs(trans_t[0]), 0.15*abs(trans_t[0]))

            # 对eul[2]进行限制
            eul[2] = np.clip(eul[2], -0.17*abs(trans_t[0]),
                             0.17*abs(trans_t[0]))
            quat_t = tf.transformations.quaternion_from_euler(
                0.0, 0.0, eul[2])

            userdata.goal = {'frame_id': self._source_frame,
                             'position': trans_t, 'orientation': quat_t}
            Logger.loginfo("%s: realtime goal w.r.t %s, P,Q:({0}, {1}, {2}), ({3}, {4}, {5}, {6})"
                           .format(trans_t[0], trans_t[1], trans_t[2], quat_t[0],
                                   quat_t[1], quat_t[2], quat_t[3]) % (self.name, self._source_frame))
            return 'done'
        except Exception as e:
            self.tf1_lis.clear()
            Logger.logwarn('%s: tf exception: %s' % (self.name, str(e)))

    def on_exit(self, userdata):
        self.tf1_lis.clear()
