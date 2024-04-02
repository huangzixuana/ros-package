#!/usr/bin/env python

import getpass
import sys
import time
import yaml
import rospy
import moveit_commander
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import *
from std_msgs.msg import String

class moveit(EventState):
    '''
    Publish String & Subscribe String/DiagnosticArray topic

    -- ns               string              namespace
    -- topic            stirng              topic name as input pose
    -- name             string              goal name in yaml
    -- frame_id         string              frame id as reference
    -- position         list                position [x,y,z]
    -- orientation      list                orientation [x,y,z,w]
    -- timeoff          float               time off to RECALLING
    -- timeout          float               time out as LOST

    <= PREEMPTED        return              PREEMPTED
    <= SUCCEEDED        return              SUCCEEDED
    <= ABORTED          return              ABORTED

    '''

    def __init__(self, ns='', topic='', name='', frame_id='map', position=[0, 0, 0], orientation=[0, 0, 0, 1]):
        super(moveit, self).__init__(outcomes=['SUCCEEDED', 'PREEMPTED', 'ABORTED', 'LOST'])
        self._update_pub = ProxyPublisher({'flexbe/behavior_updating': String})

        self._ns = ns
        self._topic = topic
        self._iname = name
        self._frame_id = frame_id
        self._position = position
        self._orientation = orientation

        self._goal = PoseStamped()
        if self._topic != "":
            self._sub_goal = ProxySubscriberCached({self._topic:PoseStamped})
        elif self._iname != "":
            yaml_path = rospy.get_param("~waypoints_path", "~/catkin_ws/dbparam/flexbe_waypoints.yaml")
            if yaml_path == "":
                yaml_path = "~/catkin_ws/dbparam/flexbe_waypoints.yaml"
            if yaml_path[0] == "~":
                absolute_path = "/home/" + getpass.getuser() + yaml_path[1:]
            with open(absolute_path) as f:
                self._waypoints = yaml.safe_load(f)
            if self._iname in self._waypoints:
                waypoint = self._waypoints[self._iname]
                self._goal.header.frame_id = waypoint["frame_id"]
                self._goal.pose.position.x = waypoint["pose"]["position"]["x"]
                self._goal.pose.position.y = waypoint["pose"]["position"]["y"]
                self._goal.pose.position.z = waypoint["pose"]["position"]["z"]
                self._goal.pose.orientation.x = waypoint["pose"]["orientation"]["x"]
                self._goal.pose.orientation.y = waypoint["pose"]["orientation"]["y"]
                self._goal.pose.orientation.z = waypoint["pose"]["orientation"]["z"]
                self._goal.pose.orientation.w = waypoint["pose"]["orientation"]["w"]
            else:
                rospy.logwarn("%s not in %s" % self._iname, yaml_path)
        else:
            self._goal.header.frame_id = self._frame_id
            self._goal.pose.position.x = self._position[0]
            self._goal.pose.position.y = self._position[1]
            self._goal.pose.position.z = self._position[2]
            self._goal.pose.orientation.x = self._orientation[0]
            self._goal.pose.orientation.y = self._orientation[1]
            self._goal.pose.orientation.z = self._orientation[2]
            self._goal.pose.orientation.w = self._orientation[3]

        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
        # robot = moveit_commander.RobotCommander
        # scene = moveit_commander.PlanningSceneInterface()

        self._moveit = moveit_commander.MoveGroupCommander('manipulator')
                
        self._end_effector_link = self._moveit.get_end_effector_link()
        reference_frame = 'base'
        self._moveit.set_pose_reference_frame(reference_frame)
                
        self._moveit.allow_replanning(True)
        
        self._moveit.set_goal_position_tolerance(0.001)
        self._moveit.set_goal_orientation_tolerance(0.01)
       
        self._moveit.set_max_acceleration_scaling_factor(0.5)
        self._moveit.set_max_velocity_scaling_factor(0.5)

        self._time_publish = rospy.Time.now()
        self._time_enter = self._time_publish
        self._time_execute = self._time_publish

    def on_enter(self, userdata):
        '''subscribe'''
        self._time_enter = rospy.Time.now()

        update_msg = String()
        update_msg.data = self.name
        self._update_pub.publish('flexbe/behavior_updating', update_msg)

        if self._topic != "":
            self._sub_goal.enable_buffer(self._topic)
            if self._sub_goal.has_msg(self._topic):
                self._goal = self._sub_goal.get_last_msg(self._topic)
                self._sub_goal.remove_last_msg(self._topic)
        self._goal.header.stamp = self._time_enter

        try:
            self._moveit.set_start_state_to_current_state()
            
            self._moveit.set_pose_target(self._goal, self._end_effector_link)
            
            traj = self._moveit.plan()
            self._moveit.execute(traj[1], wait=True)
            # self._moveit.execute(traj, wait=False))
            # self._moveit.go()
            # self._moveit.go(wait=False)

            rospy.sleep(1)

            self._time_publish = rospy.Time.now()

        except Exception as e:
            print(e)

    def execute(self, userdata):
        '''main loop'''
        self._time_execute = rospy.Time.now()

        # print self._moveit.get_current_pose()
        return 'SUCCEEDED'

        # active_joints = self._moveit.get_active_joints()
        # for joint in active_joints:
        #     Logger.logwarn(joint)

        # if self._client.has_result(self._action_topic):
        #     Logger.logwarn("haha1")
        #     status = self._client.get_state(self._action_topic)
        #     Logger.logwarn("haha2: %d", status)
        #     if status == GoalStatus.SUCCEEDED:
        #         update_msg = String()
        #         update_msg.data = "goal_reached_%s" % self.name
        #         self._update_pub.publish('flexbe/behavior_updating', update_msg)
        #         return 'SUCCEEDED'
        #     elif status in [GoalStatus.PREEMPTED]:
        #         Logger.logwarn('%s: moveit canceled: %s' % (self.name, str(status)))
        #         return 'PREEMPTED'
        #     elif status in [GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.ABORTED]:
        #         Logger.logwarn('%s: moveit failed: %s' % (self.name, str(status)))
        #         return 'ABORTED'

    def on_exit(self, userdata):
        '''unsubscribe'''
        if self._topic != "":
            self._sub.disable_buffer(self._topic)

        self._moveit.stop()
        self._moveit.clear_pose_targets()

    def on_pause(self):
        self._moveit.stop()
