#!/usr/bin/env python

import getpass
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from flexbe_core.proxy import ProxyPublisher
import yaml
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import *
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
from std_msgs.msg import String
import rospy

class waypoint(EventState):
    '''
    move_base action client

    -- topic            stirng              action topic name, move_base as default
    -- name             string              waypoint site name, could include {0} {1} expression
    -- site_index       string/int/float    replace {0} within site name
    -- site_floor       string              replace {1} within site name, but split 'F' first
    -- timeoff          float               unused
    -- timeout          float               unused

    <= done   navigate to waypoint site
    '''
    def __init__(self, topic='move_base', name='', site_index='', site_floor='', timeoff=1.0, timeout=0.0):
        super(waypoint, self).__init__(outcomes=['arrived', 'canceled', 'failed'],
                                       input_keys=['topic', 'name', 'site_index', 'site_floor'])
        self._update_pub = ProxyPublisher({'flexbe/behavior_updating': String})

        self._topic = topic
        self._site = {}
        self._site['name'] = name
        self._site['index'] = site_index
        self._site['floor'] = site_floor
        self._timeoff = timeoff
        self._timeout = timeout

        if self._topic != '' and self._topic != 'null':
            if self._topic == 'initialpose':
                self._pub = ProxyPublisher({self._topic: PoseWithCovarianceStamped})
            else:
                self._client = ProxyActionClient({self._topic: MoveBaseAction})

        yaml_path = rospy.get_param(
            "~waypoints_path", "~/catkin_ws/dbparam/flexbe_waypoints.yaml")
        if yaml_path != "":
            if yaml_path[0] == '~':
                absolute_path = '/home/' + getpass.getuser() + yaml_path[1:]
            with open(absolute_path) as f:
                self._waypoints = yaml.safe_load(f)

    def execute(self, userdata):
        """Wait for action result and return outcome accordingly"""
        if self._topic == 'initialpose':
            return 'arrived'
        if self._client.has_result(self._topic):
            status = self._client.get_state(self._topic)
            if status == GoalStatus.SUCCEEDED:
                return 'arrived'
            elif status in [GoalStatus.PREEMPTED]:
                Logger.logwarn('navigation canceled: %s' % str(status))
                return 'canceled'
            elif status in [
                    GoalStatus.REJECTED, GoalStatus.RECALLED,
                    GoalStatus.ABORTED
            ]:
                Logger.logwarn('navigation failed: %s' % str(status))
                return 'failed'

    def on_enter(self, userdata):
        if self._topic == '':
            self._topic = userdata.topic
            if self._topic != '' and self._topic != 'null':
                if self._topic == 'initialpose':
                    self._pub = ProxyPublisher({self._topic: PoseWithCovarianceStamped})
                else:
                    self._client = ProxyActionClient({self._topic: MoveBaseAction})
        if self._site['name'] == '':
            self._site['name'] = userdata.name
        if self._site['index'] == '':
            self._site['index'] = userdata.site_index
        if self._site['floor'] == '':
            self._site['floor'] = userdata.site_floor

        split_floor = self._site['floor'].find('F')
        floor = self._site['floor'][:split_floor]
        self._site_name = self._site['name'].format(self._site['index'], floor)
        self._nav_site = self._waypoints[self._site_name]

        update_msg = String()
        update_msg.data = self._site_name
        self._update_pub.publish('flexbe/behavior_updating', update_msg)

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self._nav_site['frame_id']
        pose = Pose()
        pose.position.x = self._nav_site['pose']['position']['x']
        pose.position.y = self._nav_site['pose']['position']['y']
        pose.position.z = self._nav_site['pose']['position']['z']
        pose.orientation = Quaternion(
            self._nav_site['pose']['orientation']['x'],
            self._nav_site['pose']['orientation']['y'],
            self._nav_site['pose']['orientation']['z'],
            self._nav_site['pose']['orientation']['w'])

        if self._topic == 'initialpose':
            initial_msg = PoseWithCovarianceStamped()
            initial_msg.header = header
            initial_msg.pose.pose = pose
            self._pub.publish(self._topic, initial_msg)
        else:
            mv_goal = MoveBaseActionGoal()
            mv_goal.goal.target_pose.header = header
            mv_goal.goal.target_pose.pose = pose
            try:
                self._client.send_goal(self._topic, mv_goal.goal)
                Logger.loginfo(
                    "send navigation goal with position:({0}, {1}, {2}) and  orientation: ({3}, {4}, {5}, {6})"
                    .format(self._nav_site['pose']['position']['x'],
                            self._nav_site['pose']['position']['y'],
                            self._nav_site['pose']['position']['z'],
                            self._nav_site['pose']['orientation']['x'],
                            self._nav_site['pose']['orientation']['y'],
                            self._nav_site['pose']['orientation']['z'],
                            self._nav_site['pose']['orientation']['w']))
            except Exception as e:
                Logger.logwarn('Failed to connect to movebase server')

    def cancel_active_goals(self):
        if self._client.is_available(self._topic):
            if self._client.is_active(self._topic):
                if not self._client.has_result(self._topic):
                    self._client.cancel(self._topic)
                    Logger.loginfo('Canceling navigation active goal.')

    def on_exit(self, userdata):
        if self._topic != 'initialpose':
            self.cancel_active_goals()

    def on_pause(self):
        if self._topic != 'initialpose':
            self.cancel_active_goals()

