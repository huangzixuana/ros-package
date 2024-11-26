#!/usr/bin/env python

from flexbe_core import EventState, Logger
import rospy
import sys
import copy
import moveit_commander
import tf
import time
import numpy as np
from tf import TransformListener
from actionlib_msgs.msg import GoalID
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from std_msgs.msg import Bool, Empty, Header
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import OrientationConstraint, Constraints, RobotTrajectory
from sensor_msgs.msg import JointState
import yaml
import getpass
import os
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import PoseWithCovarianceStamped


class SiteManipulationCali(EventState):
    def __init__(self,
                 calibration_site = "",
                 reference_frame="base_arm",
                 end_effector_link="tool0",
                 wait_time=0,
                 v_factor=1,
                 a_factor=1,
                 if_execute=True,
                 if_debug=False):
        super(SiteManipulationCali, self).__init__(outcomes=['done', 'failed'],
                                               input_keys=['move_group', 'arm_site', 'no_site'])
        self._reference_frame, self._end_effector_link = reference_frame, end_effector_link
        self._v_factor, self._a_factor = v_factor, a_factor
        self._if_execute = if_execute
        self._wait_time = wait_time
        self._if_debug = if_debug
        self._res = ''
        self._calibration_site = calibration_site

        self._traj_pub = ProxyPublisher(
            {'position_trajectory_controller/command': JointTrajectory})
        self._traj_todo_pub = ProxyPublisher(
            {'joint_trajectory_todo': JointTrajectory})

        self.tf1_lis = TransformListener()
        self.entered = False
        self._preempted = False

        self._break_enter = False
        self._pause_sub = ProxySubscriberCached({'flexbe/command/pause': Bool})
        self._pause_sub.set_callback('flexbe/command/pause', self.pause_cb)
        self._preempt_sub = ProxySubscriberCached(
            {'flexbe/command/preempt': Empty})
        self._preempt_sub.set_callback(
            'flexbe/command/preempt', self.preempt_cb)

        self._traj_todo_rec = False
        self._traj_todo_msg = None
        self._traj_done_sub = ProxySubscriberCached(
            {'joint_trajectory_done': Header})
        self._traj_done_sub.subscribe('joint_trajectory_done', Header,
                                      callback=self.traj_done_cb, buffered=False)

        if self._if_debug:
            self.init_data()
            self._joint_state_sub = ProxySubscriberCached(
                {'joint_states': JointState})
            self._controller_state_sub = ProxySubscriberCached(
                {'position_trajectory_controller/state': JointTrajectoryControllerState})
            if not os.path.isfile(self.plot_data['path']):
                with open(self.plot_data['path'], 'w') as fp:
                    pass

    def relative_to_absolute_path(self, relative_path):
        if relative_path[0] == '~':
            return '/home/' + getpass.getuser() + relative_path[1:]
        else:
            return relative_path

    def traj_done_cb(self, msg):
        self._traj_todo_msg = msg
        self._traj_todo_rec = True

    def manipulator_init(self):
        group_name = rospy.get_param(
            "/move_group/moveit_sim_hw_interface/joint_model_group", "leapting_arm")
        moveit_commander.roscpp_initialize(sys.argv)
        self._move_group = moveit_commander.MoveGroupCommander(group_name)
        if self._reference_frame != "":
            self._move_group.set_pose_reference_frame(self._reference_frame)
        if self._end_effector_link != "":
            self._move_group.set_end_effector_link(self._end_effector_link)
        Logger.loginfo('%s:self manipulation init.' % self.name)

    def on_enter(self, userdata):
        self.tf1_lis.clear()
        if userdata.move_group != None:
            self._move_group = userdata.move_group
            Logger.loginfo('%s:manipulation init by userdata.' % self.name)
        else:
            self.manipulator_init()

        if self._preempted:
            return

        self.moveit_goal_init()
        if self._if_debug:
            self.init_data()
            Logger.loginfo("debug file path: %s" % self.plot_data['path'])

        self.entered = True
        self._res = 'failed'

        self._move_group.set_start_state_to_current_state()
        
        site_name = 'calibration_'+str(userdata.no_site)
        if self._calibration_site != "":
            site_name = 'calibration_'+self._calibration_site
        site_joints = userdata.arm_site[site_name]
        Logger.loginfo('%s: target is %s' % (self.name, site_name))
        print(site_joints)
        # site_joints_float = [float(x) for x in site_joints]
        self._move_group.set_joint_value_target(site_joints)

        Logger.loginfo('%s: planning' % self.name)
        plan_msg = self._move_group.plan()

        if rospy.get_param("/rosdistro")[:7] == 'melodic':
            if plan_msg.joint_trajectory.points != []:
                Logger.loginfo("%s: path planned" % self.name)
                if self._if_execute:
                    if self._move_group.execute(plan_msg, wait=True):
                        self._res = 'done'
                        Logger.loginfo('%s succeed: executed' % self.name)
                    else:
                        Logger.logwarn('%s failed: executed' % self.name)
                else:
                    self._res = 'done'
            else:
                Logger.logwarn("%s failed:  no path" % self.name)
        elif rospy.get_param("/rosdistro")[:6] == 'noetic':
            if plan_msg[0] and self.check_plan_step(plan_msg[1]):
                Logger.loginfo("%s: path planned" % self.name)
                if self._if_execute:
                    if self._move_group.execute(plan_msg[1], wait=True):
                        self._res = 'done'
                        Logger.loginfo('%s succeed: executed' % self.name)
                    else:
                        Logger.logwarn('%s failed: executed' % self.name)
                else:
                    self._res = 'done'
            else:
                Logger.logwarn("%s:  no valid path" % self.name)

    def init_data(self):
        self.plot_data = {
            'path': '/home/' + getpass.getuser()+"/.ros/site_manipulation.yaml",
            'joints': [],
            'plan': {'position': [], 'velocities': [], 'accelerations': [], 'time': []},
            'real': {'position': [], 'velocities': [], 'effort': [], 'time': []},
            'last_joint': 0,
            'ts': 0,
            'enter_time': time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())),
            'controller_write': {'position': [], 'velocities': [], 'accelerations': [], 'time': []},
            'controller_read': {'position': [], 'velocities': [], 'accelerations': [], 'time': []},
            'controller_error': {'position': [], 'velocities': [], 'accelerations': [], 'time': []}
        }
        self.controller_msg = JointTrajectoryControllerState()

    def controller_cb(self, msg):
        self.controller_msg = msg

    def joints_cb(self, msg):
        pos = msg.position
        if self.plot_data['last_joint'] == 0 or self.joint_joint_diff(pos, self.plot_data['last_joint']):
            self.plot_data['real']['time'].append(
                round(time.time() - self.plot_data['ts'], 2))
            self.plot_data['real']['position'].append(list(pos))
            self.plot_data['real']['velocities'].append(list(msg.velocity))
            self.plot_data['real']['effort'].append(list(msg.effort))
            self.plot_data['last_joint'] = list(pos)

            if list(self.controller_msg.desired.positions) != []:
                self.plot_data['controller_write']['time'].append(
                    round(self.controller_msg.desired.time_from_start.to_sec(), 2))
                self.plot_data['controller_write']['position'].append(
                    list(self.controller_msg.desired.positions))
                self.plot_data['controller_write']['velocities'].append(
                    list(self.controller_msg.desired.velocities))
                self.plot_data['controller_write']['accelerations'].append(
                    list(self.controller_msg.desired.accelerations))

                self.plot_data['controller_read']['time'].append(
                    round(self.controller_msg.actual.time_from_start.to_sec(), 2))
                self.plot_data['controller_read']['position'].append(
                    list(self.controller_msg.actual.positions))
                self.plot_data['controller_read']['velocities'].append(
                    list(self.controller_msg.actual.velocities))
                self.plot_data['controller_read']['accelerations'].append(
                    list(self.controller_msg.actual.accelerations))

                self.plot_data['controller_error']['time'].append(
                    round(self.controller_msg.error.time_from_start.to_sec(), 2))
                self.plot_data['controller_error']['position'].append(
                    list(self.controller_msg.error.positions))
                self.plot_data['controller_error']['velocities'].append(
                    list(self.controller_msg.error.velocities))
                self.plot_data['controller_error']['accelerations'].append(
                    list(self.controller_msg.error.accelerations))

            with open(self.plot_data['path'], "w") as f:
                yaml.dump(self.plot_data, f)
        else:
            pass

    def joint_joint_diff(self, pos1, pos2, d=0.05):
        for j in range(len(pos1)):
            if abs(pos1[j] - pos2[j]) > d:
                return True
        return False

    def check_plan_step(self, plan, d=1.5):
        plan_points = plan.joint_trajectory.points
        if len(plan_points) < 2:
            rospy.logwarn('plan point < 2')
            return True

        t_l = list(
            map(lambda p: round(p.time_from_start.to_sec(), 2), plan_points))
        dt = [t_l[i+1] - t_l[i] for i in range(len(t_l)-1)]
        dt.sort(reverse=True)

        if len(dt) >= 2 and dt[0] > 3.0*dt[1]:
            Logger.logwarn("%s failed: plan time step too large" % self.name)
            return False

        for i in range(len(plan_points)-1):
            pos1 = np.array(list(plan_points[i].positions))
            pos2 = np.array(list(plan_points[i+1].positions))
            max_diff = np.max(np.abs(pos1-pos2))
            if max_diff > d:
                Logger.logwarn("%s failed: plan joint step max diff: {0}".format(
                    max_diff) % self.name)
                return False

        return True

    def execute(self, userdata):
        return self._res

    def pause_cb(self, msg):
        if msg.data:
            self.cancel_goal()

    def preempt_cb(self, msg):
        self.cancel_goal()
        self._preempted = True

    def cancel_goal(self):
        if self.entered:
            # self._cancel_pub.publish(
            #     'position_trajectory_controller/follow_joint_trajectory/cancel', GoalID())

            self._traj_pub.publish(
                'position_trajectory_controller/command', JointTrajectory())
            Logger.loginfo("%s:  cancel goal" % self.name)
        self._break_enter = True

    def on_exit(self, userdata):
        self.entered = False
        time.sleep(self._wait_time)

        self._move_group.stop()
        self._move_group.clear_pose_targets()
        if self._if_debug:
            self._joint_state_sub.unsubscribe_topic('joint_states')
            self._controller_state_sub.unsubscribe_topic(
                'position_trajectory_controller/state')
        Logger.loginfo("-"*20)

    def moveit_goal_init(self):
        self._break_enter = False
        self._move_group.clear_pose_targets()
        self._move_group.set_planning_time(5)
        self._move_group.allow_replanning(True)

        self._move_group.set_max_velocity_scaling_factor(self._v_factor)
        self._move_group.set_max_acceleration_scaling_factor(self._a_factor)
        # self._move_group.set_goal_tolerance(0.01)
        # self._move_group.set_goal_orientation_tolerance(0.01)
