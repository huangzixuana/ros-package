#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient, ProxySubscriberCached
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import *
from geometry_msgs.msg import Quaternion, Pose, Point
import tf
import time
import yaml
import getpass
import rospy
import copy
import numpy as np


'''
Created on 01.06.2020
@author: lei.zeng@tu-dortmund.de
'''


class SiteNavigation(EventState):
    '''
    Navigation to desired site
    -- site_name	         string       site name collected in yaml 
    -- position              list         position [x,y,z]
    -- orientation           list         orientation [x,y,z,w]
    -- frame_id              string       frame id as reference
    -- base_link2map         bool

    <= arrived                  arrived
    <= failed                   failed
    <= canceled                canceled
    '''

    def __init__(self, site_name="", position=[0, 0, 0], orientation=[0, 0, 0, 1], frame_id="map", base_link2map=False):
        super(SiteNavigation, self).__init__(
            outcomes=['arrived', 'canceled', 'failed'], input_keys=["nav_goal"])
        self._site_name = site_name
        self._position = position
        self._orientation = orientation
        self._frame_id = frame_id
        self._base_link2map = base_link2map
        self._action_topic = "move_base"

        self._init_done = False
        rospy.Timer(rospy.Duration(0.1), self.time_cb, oneshot=True)

        self._pose_sub = ProxySubscriberCached(
            {'robot_pose': Pose})
        self._pose_sub.subscribe('robot_pose', Pose,
                                 callback=self.pose_cb, buffered=False)
        self._received = False
        self._pose_msg = None

        yaml_path = rospy.get_param(
            "~waypoints_path", "~/catkin_ws/dbparam/flexbe_waypoints.yaml")
        if yaml_path != "":
            if yaml_path[0] == '~':
                absolute_path = '/home/' + getpass.getuser() + yaml_path[1:]
            with open(absolute_path) as f:
                self._waypoints = yaml.safe_load(f)
        if self._site_name != "":
            if self._site_name in self._waypoints:
                self._nav_site = self._waypoints[self._site_name]
            else:
                rospy.logwarn('%s not exits' % self._site_name)
        else:
            self._nav_site = {}

    def pose_cb(self, msg):
        self._pose_msg = msg
        self._received = True

    def time_cb(self, event):
        self._client = ProxyActionClient(
            {self._action_topic: MoveBaseAction})
        self._init_done = True

    def on_enter(self, userdata):
        self._received = False

        while (not self._init_done):
            time.sleep(0.02)

        while ((not self._received) and self._base_link2map):
            time.sleep(0.02)

        try:
            mv_goal = MoveBaseActionGoal()
            mv_goal.goal.target_pose.header.stamp = rospy.Time.now()
            mv_goal_pose = mv_goal.goal.target_pose.pose
            if self._site_name != "":
                Logger.loginfo('%s: goal site is %s' %
                               (self.name, self._site_name))
                mv_goal.goal.target_pose.header.frame_id = self._nav_site[
                    'frame_id']
                nav_pos = copy.deepcopy(self._nav_site['pose']['position'])
                nav_ort = copy.deepcopy(
                    self._nav_site['pose']['orientation'])
            elif userdata.nav_goal != {}:
                Logger.loginfo('%s: userdata goal' % self.name)
                mv_goal.goal.target_pose.header.frame_id = userdata.nav_goal["frame_id"]
                nav_pos = {'x': userdata.nav_goal["position"][0],
                           'y': userdata.nav_goal["position"][1], 'z': userdata.nav_goal["position"][2]}
                nav_ort = {'x': userdata.nav_goal["orientation"][0], 'y': userdata.nav_goal["orientation"][1],
                           'z': userdata.nav_goal["orientation"][2], 'w': userdata.nav_goal["orientation"][3]}

            else:
                Logger.loginfo('%s: flexbe param goal' % self.name)
                mv_goal.goal.target_pose.header.frame_id = self._frame_id
                nav_pos = {'x': self._position[0],
                           'y': self._position[1], 'z': self._position[2]}
                nav_ort = {'x': self._orientation[0], 'y': self._orientation[1],
                           'z': self._orientation[2], 'w': self._orientation[3]}

            if self._base_link2map and mv_goal.goal.target_pose.header.frame_id == "base_link":
                Logger.loginfo('convert goal from base_link2map')
                Tm_b = tf.TransformerROS().fromTranslationRotation([
                    self._pose_msg.position.x,
                    self._pose_msg.position.y,
                    self._pose_msg.position.z
                ], [
                    self._pose_msg.orientation.x,
                    self._pose_msg.orientation.y,
                    self._pose_msg.orientation.z,
                    self._pose_msg.orientation.w,
                ])

                Tb_t = tf.TransformerROS().fromTranslationRotation([
                    nav_pos['x'],
                    nav_pos['y'],
                    nav_pos['z']
                ], [
                    nav_ort['x'],
                    nav_ort['y'],
                    nav_ort['z'],
                    nav_ort['w'],
                ])

                Tm_t = np.dot(Tm_b, Tb_t)
                trans_t = list(
                    tf.transformations.translation_from_matrix(Tm_t))
                eul = list(tf.transformations.euler_from_matrix(Tm_t))
                quat_t = tf.transformations.quaternion_from_euler(
                    0.0, 0.0, eul[2])

                mv_goal.goal.target_pose.header.frame_id = "map"
                mv_goal_pose.position = Point(
                    trans_t[0], trans_t[1], trans_t[2])
                mv_goal_pose.orientation = Quaternion(quat_t[0], quat_t[1],
                                                      quat_t[2], quat_t[3])

            else:
                mv_goal_pose.position = Point(
                    nav_pos['x'], nav_pos['y'], nav_pos['z'])
                mv_goal_pose.orientation = Quaternion(nav_ort['x'], nav_ort['y'],
                                                      nav_ort['z'], nav_ort['w'])

            self._client.send_goal(self._action_topic, mv_goal.goal)
            Logger.loginfo("%s: send navigation goal w.r.t %s: P=({0}, {1}, {2}) and O=({3}, {4}, {5}, {6})"
                           .format(nav_pos['x'], nav_pos['y'], nav_pos['z'], nav_ort['x'], nav_ort['y'],
                                   nav_ort['z'], nav_ort['w']) % (self.name,  mv_goal.goal.target_pose.header.frame_id))
        except Exception as e:
            Logger.logwarn('%s: Failed to connect to move_base server' %
                           self.name)

    def execute(self, userdata):
        if self._client.has_result(self._action_topic):
            status = self._client.get_state(self._action_topic)
            if status == GoalStatus.SUCCEEDED:
                return 'arrived'
            elif status in [GoalStatus.PREEMPTED]:
                Logger.logwarn('%s: navigation canceled: %s' %
                               (self.name, str(status)))
                return 'canceled'
            elif status in [GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.ABORTED]:
                Logger.logwarn('%s: navigation failed: %s' %
                               (self.name, str(status)))
                return 'failed'

    def cancel_active_goals(self):
        if self._client.is_available(self._action_topic):
            if self._client.is_active(self._action_topic):
                if not self._client.has_result(self._action_topic):
                    self._client.cancel(self._action_topic)
                    Logger.loginfo('%s: Canceling navigation active goal.' %
                                   self.name)

    def on_stop(self):
        self.cancel_active_goals()

    def on_pause(self):
        self.cancel_active_goals()
