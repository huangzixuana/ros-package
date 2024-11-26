#!/usr/bin/env python
from flexbe_core import EventState, Logger
import tf
from tf import TransformListener
import time
import rospy
import numpy as np
from flexbe_core.proxy import ProxySubscriberCached, ProxyPublisher
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from nav_msgs.msg import Path
import yaml
import os
import getpass
import copy
from moveit_commander import MoveGroupCommander
'''
Created on Jun 5th, 2023
@author: lei.zeng@leapting.com, chenxin.zou@leapting.com ,lei.wu@leapting.com
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

    def __init__(self, position=[0, 0, 0],
                 orientation=[0, 0, 0.7071, 0.7071],
                 frame_id='', source_frame='map',
                 if_back=False,
                 gap_pvm_pvm=0.35,
                 gap_pvm_chassis=0,
                 chassis_width=2.2,
                 k_y=0.1,
                 k_yaw=0.1,
                 use_solar=True):
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

        self._k_y = k_y
        self._k_yaw = k_yaw

        self._use_solar = use_solar

        self.tf1_lis = TransformListener()

        self._solar_pose_sub = ProxySubscriberCached(
            {'filter_solar_pose': PoseWithCovarianceStamped})
        self._solar_pose_sub.subscribe('filter_solar_pose', PoseWithCovarianceStamped,
                                       callback=self.solar_cb, buffered=False)
        self._received = False
        self._solar_msg = None
        self._pose_sub = ProxySubscriberCached(
            {'robot_pose': Pose})

        self.before_limit = ProxyPublisher({'before_limit_goal': PoseStamped})
        self.yaml_path = self.relative_to_absolute_path(
            '~/catkin_ws/src/test.yaml')
        self.history_pub = ProxyPublisher({'history_msg': Path})
        self.yaml_solar_path = self.relative_to_absolute_path(
            '~/catkin_ws/src/test_solar.yaml')
        self.history_solar_pub = ProxyPublisher({'history_solar_msg': Path})

        self.move_group = MoveGroupCommander('leapting_arm')

    def solar_cb(self, msg):
        self._solar_msg = msg
        self._received = True

    def on_enter(self, userdata):
        self.tf1_lis.clear()
        self._received = False

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
            T1 = None
            T0 = tf.TransformerROS().fromTranslationRotation(
                [0, 0, 0], [0, 0, 1, 0])
            if self._use_solar:
                if self._received:
                    p = [self._solar_msg.pose.pose.position.x,
                         self._solar_msg.pose.pose.position.y,
                         self._solar_msg.pose.pose.position.z]

                    q = [self._solar_msg.pose.pose.orientation.x,
                         self._solar_msg.pose.pose.orientation.y,
                         self._solar_msg.pose.pose.orientation.z,
                         self._solar_msg.pose.pose.orientation.w]
                    T1 = tf.TransformerROS().fromTranslationRotation(p, q)

                else:
                    return
            else:
                self.move_group.set_end_effector_link("tool0")
                tool_pose = self.move_group.get_current_pose()
                if tool_pose.header.frame_id == "base_arm":

                    p = [tool_pose.pose.position.x,
                         tool_pose.pose.position.y,
                         tool_pose.pose.position.z]

                    q = [tool_pose.pose.orientation.x,
                         tool_pose.pose.orientation.y,
                         tool_pose.pose.orientation.z,
                         tool_pose.pose.orientation.w]
                    T1 = tf.TransformerROS().fromTranslationRotation(p, q)
                else:
                    Logger.loginfo(
                        "need transform current end_effector_link frame_id to base_arm")
                    return
            T1 = np.dot(T0, T1)
            T2 = tf.TransformerROS().fromTranslationRotation(
                self._position, self._orientation)
            T = np.dot(T1, T2)

            t1 = tf.transformations.translation_from_matrix(T1)
            q1 = tf.transformations.quaternion_from_matrix(T1)

            Logger.loginfo("T base_arm solar_link: P,Q:({0:.2f}, {1:.2f}, {2:.2f}), ({3:.2f}, {4:.2f}, {5:.2f}, {6:.2f})"
                           .format(p[0], p[1], p[2], q[0], q[1], q[2], q[3]))
            Logger.loginfo("T1 base_link solar_link: P,Q:({0:.2f}, {1:.2f}, {2:.2f}), ({3:.2f}, {4:.2f}, {5:.2f}, {6:.2f})"
                           .format(t1[0], t1[1], t1[2], q1[0], q1[1], q1[2], q1[3]))
            Logger.loginfo("T2 solar_link target: P,Q:({0:.2f}, {1:.2f}, {2:.2f}), ({3:.2f}, {4:.2f}, {5:.2f}, {6:.2f})"
                           .format(self._position[0], self._position[1], self._position[2],
                                   self._orientation[0], self._orientation[1], self._orientation[2], self._orientation[3]))

            trans_t = list(tf.transformations.translation_from_matrix(T))
            eul = list(tf.transformations.euler_from_matrix(T))
            # eul = list(tf.transformations.euler_from_matrix(T, axes= 'rxyz'))

            # before limitation
            Logger.loginfo("before limitation: nav P,Eul:({0:.2f}, {1:.2f}, {2:.2f}), ({3:.2f}, {4:.2f}, {5:.2f})"
                           .format(trans_t[0], trans_t[1], trans_t[2], eul[0], eul[1], eul[2]))

            qtemp = list(tf.transformations.quaternion_from_matrix(T))
            before_goal = PoseStamped()
            before_goal.header.frame_id = "base_link"
            before_goal.header.stamp = rospy.Time.now()
            before_goal.pose.position.x = trans_t[0]
            before_goal.pose.position.y = trans_t[1]
            before_goal.pose.position.z = trans_t[2]
            before_goal.pose.orientation.x = qtemp[0]
            before_goal.pose.orientation.y = qtemp[1]
            before_goal.pose.orientation.z = qtemp[2]
            before_goal.pose.orientation.w = qtemp[3]
            self.before_limit.publish('before_limit_goal', before_goal)

            if self._if_back:
                trans_t[0] = np.clip(trans_t[0], -2, 0)
            else:
                trans_t[0] = np.clip(trans_t[0], 0, 3)

            trans_t[1] = self._k_y * trans_t[1]
            trans_t[2] = self._k_y * trans_t[2]
            eul[0] = self._k_yaw * eul[0]
            eul[1] = self._k_yaw * eul[1]
            eul[2] = self._k_yaw * eul[2]
            quat_t = tf.transformations.quaternion_from_euler(
                eul[0], eul[1], eul[2])

            if self._pose_sub.has_msg('robot_pose'):
                pose_msg = self._pose_sub.get_last_msg(
                    'robot_pose')
                Tm_b = tf.TransformerROS().fromTranslationRotation([
                    pose_msg.position.x,
                    pose_msg.position.y,
                    pose_msg.position.z
                ], [
                    pose_msg.orientation.x,
                    pose_msg.orientation.y,
                    pose_msg.orientation.z,
                    pose_msg.orientation.w,
                ])
                Tb_t = tf.TransformerROS().fromTranslationRotation(trans_t, quat_t)
                Tm_t = np.dot(Tm_b, Tb_t)
                trans_t_float_temp = tf.transformations.translation_from_matrix(
                    Tm_t)
                quat_t_float_temp = tf.transformations.quaternion_from_matrix(
                    Tm_t)
                if self._source_frame == "map":
                    # eul_map = list(tf.transformations.euler_from_matrix(Tm_t,axes='rxyz'))
                    eul_map = list(tf.transformations.euler_from_matrix(Tm_t))
                    quat_map = tf.transformations.quaternion_from_euler(
                        0.0, 0.0, eul_map[2])
                    trans_map = copy.deepcopy(trans_t_float_temp)
                    trans_map[2] = 0.0
                trans_t_float = [float(x) for x in trans_t_float_temp]
                quat_t_float = [float(x) for x in quat_t_float_temp]
                record_dic = {'pos': trans_t_float, 'quat': quat_t_float}
                self.record_yaml(self.path, record_dic, self.yaml_path)
                goal_msg = self.adjust_value(self.path, 10, self.yaml_path)
                self.history_pub.publish('history_msg', goal_msg)
                Tm_s = np.dot(Tm_b, T1)
                ps = tf.transformations.translation_from_matrix(
                    Tm_s)
                qs = tf.transformations.quaternion_from_matrix(
                    Tm_s)
                ps_float = [float(x) for x in ps]
                qs_float = [float(x) for x in qs]
                solar_dic = {'pos': ps_float, 'quat': qs_float}
                self.record_yaml(self.path, solar_dic, self.yaml_solar_path)
                solar_path_msg = self.adjust_value(
                    self.path, 10, self.yaml_solar_path)
                self.history_solar_pub.publish(
                    'history_solar_msg', solar_path_msg)
            else:
                return

            if self._source_frame == "map":
                trans_t = trans_map
                quat_t = quat_map

            # eul_t = list(tf.transformations.euler_from_quaternion([quat_t[0], quat_t[1], quat_t[2], quat_t[3]]))
            # eul_t[2] = self._k_yaw * eul_t[2]
            # trans_t[1] = self._k_y * trans_t[1]
            # quat_t = tf.transformations.quaternion_from_euler(eul_t[0], eul_t[1], eul_t[2])

            userdata.goal = {'frame_id': self._source_frame,
                             'position': trans_t, 'orientation': quat_t}
            Logger.loginfo("%s: realtime goal w.r.t %s, P,Q:({0:.2f}, {1:.2f}, {2:.2f}), ({3:.2f}, {4:.2f}, {5:.2f}, {6:.2f})"
                           .format(trans_t[0], trans_t[1], trans_t[2], quat_t[0],
                                   quat_t[1], quat_t[2], quat_t[3]) % (self.name, self._source_frame))
            return 'done'
        except Exception as e:
            self.tf1_lis.clear()
            Logger.logwarn('%s: tf exception: %s' % (self.name, str(e)))

    def on_exit(self, userdata):
        self.tf1_lis.clear()

    def on_start(self):
        try:
            if not os.path.exists(self.yaml_path):
                fw = open(self.yaml_path, 'w')
                fw.close()
            else:
                fr = open(self.yaml_path, 'r')
                dat = yaml.safe_load(fr)
                if dat is not None:
                    for k in dat.keys():
                        dat[k] = {}
                fw = open(self.yaml_path, 'w')
                yaml.dump(dat, fw)
                fw.close()
            if not os.path.exists(self.yaml_solar_path):
                fw = open(self.yaml_solar_path, 'w')
                fw.close()
            else:
                fr = open(self.yaml_solar_path, 'r')
                dat = yaml.safe_load(fr)
                if dat is not None:
                    for k in dat.keys():
                        dat[k] = {}
                fw = open(self.yaml_solar_path, 'w')
                yaml.dump(dat, fw)
                fw.close()
        except Exception as e:
            Logger.logwarn('%s: clear exception: %s' % (self.name, str(e)))

    def record_yaml(self, key, dic, file_path):
        try:
            fr = open(file_path, 'r')
            dat = yaml.safe_load(fr)
            if dat is None:
                dat = {}
                d = {1: dic}
                dat[key] = d
            else:
                if key in dat.keys():
                    kn = len(dat[key].keys())
                    dat[key][kn+1] = dic
                else:
                    d = {1: dic}
                    dat[key] = d
            fw = open(file_path, 'w')
            yaml.dump(dat, fw)
            fw.close()
        except:
            pass

    def adjust_value(self, key, history_num, file_path):
        fr = open(file_path, 'r')
        dat = yaml.safe_load(fr)
        knum = len(dat[key].keys())
        kv = [x for x in range(knum, knum-history_num, -1) if x > 0]
        posl = []
        quatl = []
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        for k in kv:
            posl.append(dat[key][k]['pos'][:])
            quatl.append(dat[key][k]['quat'][:])

            pose_temp = PoseStamped()
            pose_temp.pose.position.x = dat[key][k]['pos'][0]
            pose_temp.pose.position.y = dat[key][k]['pos'][1]
            pose_temp.pose.position.z = dat[key][k]['pos'][2]
            pose_temp.pose.orientation.x = dat[key][k]['quat'][0]
            pose_temp.pose.orientation.y = dat[key][k]['quat'][1]
            pose_temp.pose.orientation.z = dat[key][k]['quat'][2]
            pose_temp.pose.orientation.w = dat[key][k]['quat'][3]
            path_msg.poses.append(pose_temp)
        return path_msg
        # self.history_pub.publish('history_msg', path_msg)

    def relative_to_absolute_path(self, relative_path):
        if relative_path[0] == '~':
            return '/home/' + getpass.getuser() + relative_path[1:]
        else:
            return relative_path
