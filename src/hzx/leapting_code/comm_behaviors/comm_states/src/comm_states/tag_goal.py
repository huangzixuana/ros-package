#!/usr/bin/env python
from flexbe_core import EventState, Logger
import tf
from tf import TransformListener
import rospy
from std_msgs.msg import Bool, Empty
import numpy as np
from flexbe_core.proxy import ProxySubscriberCached
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
import copy
import yaml
import getpass
from apriltag_ros.msg import AprilTagDetectionArray

'''
Created on May 15, 2024
@author: lei.zeng@leapting.com
'''


class TagGoal(EventState):
    '''
    Dynamically generate goal and convert reference frame

    -- pos              list         position [x,y,z]
    -- quat             list         orientation [x,y,z,w]

    #> pose_msg  PoseWithCovarianceStamped    pose_msg

    <= done                  done
    '''

    def __init__(self, pos=[0, 0, 0],
                 quat=[0, 0, 0, 1],
                 limit_dict={},
                 pos_targets=[],
                 itp_norm=0.0):
        super(TagGoal, self).__init__(
            outcomes=['done'],
            input_keys=['pose_msg'],
            output_keys=['mani_goal'])
        self._pos, self._quat = pos, quat

        self._pos_targets = pos_targets
        self._itp_norm = itp_norm

        self._limit_dict = limit_dict

        self._tag_msg = None
        self._tag_rec = False
        self._tag_sub = ProxySubscriberCached(
            {'tag_pose': Pose})
        self._tag_sub.subscribe('tag_pose', Pose,
                                callback=self.tag_cb, buffered=False)

        self._tag_det_msg = None
        self._tag_det_rec = False
        self._tag_det_sub = ProxySubscriberCached(
            {'tag_detections': AprilTagDetectionArray})
        self._tag_det_sub.subscribe('tag_detections', AprilTagDetectionArray,
                                    callback=self.tag_det_cb, buffered=False)

    def tag_det_cb(self, msg):
        self._tag_det_msg = msg
        self._tag_det_rec = True

    def tag_cb(self, msg):
        self._tag_msg = msg
        self._tag_rec = True

    def pose2mat(self, pose_msg):
        p, q = self.from_pose_msg_to_pos_quat(pose_msg)
        return tf.TransformerROS().fromTranslationRotation(p, q)

    def pose2mat2(self, pose_msg):
        p, q = self.from_pose_msg_to_pos_quat2(pose_msg)
        return tf.TransformerROS().fromTranslationRotation(p, q)

    def on_enter(self, userdata):
        self._tag_rec = False
        self._tag_det_rec = False

    def execute(self, userdata):
        if not self._tag_rec:
            return
        if not self._tag_det_rec:
            return

        len_std_tag = len(list(filter(lambda t: len(t.id) == 1,
                                 self._tag_det_msg.detections)))
        if len_std_tag < 2:
            Logger.logwarn("tag < 2")
            return

        pos_copy = copy.deepcopy(self._pos)

        p1, q1 = self.from_pose_msg_to_pos_quat(userdata.pose_msg)
        Logger.loginfo("T_b_slr:({0:.3f}, {1:.3f}, {2:.3f}), ({3:.3f}, {4:.3f}, {5:.3f},{6:.3f})"
                       .format(p1[0], p1[1], p1[2], q1[0], q1[1], q1[2], q1[3]))

        p_tag, q_tag = self.from_pose_msg_to_pos_quat2(self._tag_msg)
        p_tag[2] = p1[2]
        T_b_slr = tf.TransformerROS().fromTranslationRotation(p_tag, q_tag)

        T2 = tf.TransformerROS().fromTranslationRotation(pos_copy, self._quat)
        T = np.dot(T_b_slr, T2)
        trans_t = tf.transformations.translation_from_matrix(T)
        quat_t = tf.transformations.quaternion_from_matrix(T)

        if len(self._pos_targets) > 0:

            temp_solar_trans = list(
                tf.transformations.translation_from_matrix(T))
            temp_solar_quat = list(
                tf.transformations.quaternion_from_matrix(T))

            temp_itp_trans = copy.deepcopy(temp_solar_trans)
            temp_itp_trans[2] = temp_itp_trans[2]+self._itp_norm
            temp_itp_quat = temp_solar_quat
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

    def from_pose_msg_to_pos_quat2(self, pose_msg):
        pos = [pose_msg.position.x,
               pose_msg.position.y, pose_msg.position.z]
        quat = [pose_msg.orientation.x, pose_msg.orientation.y,
                pose_msg.orientation.z, pose_msg.orientation.w]
        return (pos, quat)

    def relative_to_absolute_path(self, relative_path):
        if relative_path[0] == '~':
            return '/home/' + getpass.getuser() + relative_path[1:]
        else:
            return relative_path
