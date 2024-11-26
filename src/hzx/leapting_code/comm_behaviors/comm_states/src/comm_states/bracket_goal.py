#!/usr/bin/env python
from flexbe_core import EventState, Logger
import tf
from tf import TransformListener
import rospy
from std_msgs.msg import Bool, Empty
import numpy as np
from flexbe_core.proxy import ProxySubscriberCached
from geometry_msgs.msg import PoseWithCovarianceStamped
import copy
import yaml
import getpass

'''
Created on May 15, 2024
@author: lei.zeng@leapting.com
'''


class BracketGoal(EventState):
    '''
    Dynamically generate goal and convert reference frame

    -- pos              list         position [x,y,z]
    -- quat             list         orientation [x,y,z,w]
    -- stable_time      float       wait time

    #> pose_msg  PoseWithCovarianceStamped    pose_msg

    <= done                  done
    '''

    def __init__(self, pos=[0, 0, 0],
                 quat=[0, 0, 0, 1],
                 dx_max=0.1,
                 limit_dict={},
                 pos_targets=[],
                 itp_norm=0.0):
        super(BracketGoal, self).__init__(
            outcomes=['done'],
            input_keys=['pose_msg'],
            output_keys=['mani_goal'])
        self._pos, self._quat = pos, quat
        self._dx_max = dx_max

        self._pos_targets = pos_targets
        self._itp_norm = itp_norm

        self._limit_dict = limit_dict

    def pose2mat(self, pose_msg):
        p, q = self.from_pose_msg_to_pos_quat(pose_msg)
        return tf.TransformerROS().fromTranslationRotation(p, q)

    def on_enter(self, userdata):
        pass

    def execute(self, userdata):
        pos_copy = copy.deepcopy(self._pos)
        T_b_slr = self.pose2mat(userdata.pose_msg)
        p1, q1 = self.from_pose_msg_to_pos_quat(userdata.pose_msg)
        Logger.loginfo("T_b_slr:({0:.3f}, {1:.3f}, {2:.3f}), ({3:.3f}, {4:.3f}, {5:.3f},{6:.3f})"
                        .format(p1[0], p1[1], p1[2], q1[0], q1[1], q1[2], q1[3]))

        pos_z = [0, 0, pos_copy[2]]
        T2_z = tf.TransformerROS().fromTranslationRotation(pos_z, self._quat)
        T_z = np.dot(T_b_slr, T2_z)
        trans_t_z = tf.transformations.translation_from_matrix(T_z)

        T2 = tf.TransformerROS().fromTranslationRotation(pos_copy, self._quat)
        T = np.dot(T_b_slr, T2)
        trans_t = tf.transformations.translation_from_matrix(T)
        quat_t = tf.transformations.quaternion_from_matrix(T)

        trans_t[2] = trans_t_z[2]

        if len(self._pos_targets) > 0:
            Tn_t = tf.TransformerROS().fromTranslationRotation(
                [0, 0, self._itp_norm], [0, 0, 0, 1])
            Tn_b = np.dot(T, Tn_t)
            temp_itp_trans = list(
                tf.transformations.translation_from_matrix(Tn_b))
            temp_itp_quat = list(
                tf.transformations.quaternion_from_matrix(Tn_b))
            temp_itp_trans = [float(x) for x in temp_itp_trans]
            temp_itp_quat = [float(x) for x in temp_itp_quat]
            Logger.loginfo("Tn_b calculated")

            if ('z_max' in self._limit_dict):
                temp_itp_trans[2] = min(
                    temp_itp_trans[2], self._limit_dict['z_max'])
                Logger.loginfo("Tn_b zmax limit")

            arm_site_path = self.relative_to_absolute_path(rospy.get_param(
                "~arm_site_path", "~/catkin_ws/dbparam/arm_waypoints.yaml"))
            fr = open(arm_site_path, 'r')
            arm_yaml = yaml.safe_load(fr)
            fr.close()

            arm_yaml['temp_itp'] = {
                'orientation': [round(temp_itp_quat[0], 8), round(temp_itp_quat[1], 8),
                                round(temp_itp_quat[2], 8), round(temp_itp_quat[3], 8)],
                'position': [round(temp_itp_trans[0], 8), round(temp_itp_trans[1], 8), round(temp_itp_trans[2], 8)]
            }

            temp_solar_trans = list(
                tf.transformations.translation_from_matrix(T))
            temp_solar_quat = list(
                tf.transformations.quaternion_from_matrix(T))

            temp_solar_trans = [float(x) for x in temp_solar_trans]
            temp_solar_quat = [float(x) for x in temp_solar_quat]

            arm_yaml['temp_solar'] = {
                'orientation': [round(temp_solar_quat[0], 8), round(temp_solar_quat[1], 8),
                                round(temp_solar_quat[2], 8), round(temp_solar_quat[3], 8)],
                'position': [round(temp_solar_trans[0], 8), round(temp_solar_trans[1], 8), round(temp_solar_trans[2], 8)]
            }

            fw = open(arm_site_path, 'w')
            yaml.dump(arm_yaml, fw)
            fw.close()

            userdata.mani_goal = {
                "pos": trans_t,
                "quat": quat_t,
                "target_frame": "none"
            }

        else:
            userdata.mani_goal = {
                "pos": trans_t,
                "quat": quat_t,
                "target_frame": "none"
            }

            Logger.loginfo("ud mani goal:({0:.3f}, {1:.3f}, {2:.3f}), ({3:.3f}, {4:.3f}, {5:.3f},{6:.3f})"
                           .format(trans_t[0], trans_t[1], trans_t[2], quat_t[0], quat_t[1], quat_t[2], quat_t[3]))

        return 'done'

    def from_pose_msg_to_pos_quat(self, pose_msg):
        pos = [pose_msg.pose.pose.position.x,
               pose_msg.pose.pose.position.y, pose_msg.pose.pose.position.z]
        quat = [pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y,
                pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w]
        return (pos, quat)

    def relative_to_absolute_path(self, relative_path):
        if relative_path[0] == '~':
            return '/home/' + getpass.getuser() + relative_path[1:]
        else:
            return relative_path
