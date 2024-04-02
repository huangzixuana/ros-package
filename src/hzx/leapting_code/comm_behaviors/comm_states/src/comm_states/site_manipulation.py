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
from std_msgs.msg import Bool, Empty
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import OrientationConstraint, Constraints
from sensor_msgs.msg import JointState
import yaml
import getpass
import os
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import PoseWithCovarianceStamped


'''
Updated on Oct 13th, 2023
Updated on Jun 1st, 2023
@author: lei.zeng@leapting.com
'''


class SiteManipulation(EventState):
    '''
    moveit plan and execute
    -- pos	                list            [px, py, pz]
    -- quat	                list            [ox, oy, oz, ow]
    -- target_frame         string          target frame, set "none" by default
    -- target_name          string          srdf name, leave "none" to unable
    -- axis_value           [string,float]  string can be: X, Y, Z, R, P, YAW, float is movement by target axis
    -- reference_frame      string          moveit reference_frame, use default if not set
    -- end_effector_link    string          moveit end_effector_link, use default if not set
    -- v_factor             float           max_velocity_scaling_factor
    -- a_factor             float           max_acceleration_scaling_factor
    -- if_execute           Bool            if execute the planned path or just show trajectory
    -- wait_time            float           wait time on exit


    <= done                  action complete
    <= failed                action exception
    '''

    def __init__(self, pos=[0, 0, 0],
                 quat=[0, 0, 0, 1],
                 target_frame="none",
                 target_name="none",
                 axis_value=["none", 0],
                 pos_targets=[],
                 reference_frame="base_arm",
                 end_effector_link="tool0",
                 v_factor=1,
                 a_factor=1,
                 if_execute=True,
                 wait_time=0,
                 stay_level=False,
                 cart_step_list=[3, 11],
                 retry_num=3,
                 itp_norm=0.15,
                 if_debug=False,
                 cart_limit={}):
        super(SiteManipulation, self).__init__(outcomes=['done', 'failed'],
                                               input_keys=['move_group'])

        axis_dict = {'X': 0, 'Y': 1, 'Z': 2,
                     'R': 3, 'P': 4, 'YAW': 5, 'none': -1}
        self._pos, self._quat = pos, quat
        self._target_frame, self._target_name = target_frame, target_name
        self._reference_frame, self._end_effector_link = reference_frame, end_effector_link
        self._v_factor, self._a_factor = v_factor, a_factor
        self._axis_num, self._value = axis_dict[axis_value[0]], axis_value[1]
        self._pos_targets = pos_targets

        self._if_execute = if_execute
        self._stay_level = stay_level
        self._wait_time = wait_time
        self._cart_step_list = cart_step_list
        self._retry_num = retry_num
        self._itp_norm = itp_norm
        self._if_debug = if_debug
        self._cart_limit = cart_limit
        self._res = ''

        self._traj_pub = ProxyPublisher(
            {'position_trajectory_controller/command': JointTrajectory})

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
        # self._br_target = tf.TransformBroadcaster()

        self._solar_sub = ProxySubscriberCached(
            {'filter_solar_pose': PoseWithCovarianceStamped})

        if self._if_debug:
            self.init_data()
            self._joint_state_sub = ProxySubscriberCached(
                {'joint_states': JointState})
            self._controller_state_sub = ProxySubscriberCached(
                {'position_trajectory_controller/state': JointTrajectoryControllerState})
            if not os.path.isfile(self.plot_data['path']):
                with open(self.plot_data['path'], 'w') as fp:
                    pass

        arm_site_path = self.relative_to_absolute_path(rospy.get_param(
            "~arm_site_path", "~/catkin_ws/dbparam/arm_waypoints.yaml"))
        if len(self._pos_targets) > 0:
            with open(arm_site_path) as f:
                self._waypoints = yaml.safe_load(f)
                # print(self._waypoints)

    def relative_to_absolute_path(self, relative_path):
        if relative_path[0] == '~':
            return '/home/' + getpass.getuser() + relative_path[1:]
        else:
            return relative_path

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
        pose_target = self._move_group.get_current_pose().pose
        set_target = False

        self._move_group.set_start_state_to_current_state()
        if 0:
            Logger.loginfo('%s: target is  w.r.t. frame %s' %
                           (self.name, self._target_frame))

            while (True):
                try:
                    # t = self.tf1_lis.getLatestCommonTime(self._move_group.get_end_effector_link(),
                    #                                      self._target_frame)
                    position, quaternion = self.tf1_lis.lookupTransform(
                        self._move_group.get_end_effector_link(),
                        self._target_frame, rospy.Time(0))
                    break
                except Exception as e:
                    Logger.logwarn('%s: exception tf1: %s' %
                                   (self.name, str(e)))
                    rospy.sleep(0.1)

            T1 = tf.TransformerROS().fromTranslationRotation(position, quaternion)
            T2 = tf.TransformerROS().fromTranslationRotation(self._pos, self._quat)
            T = np.dot(T1, T2)
            trans_t = tf.transformations.translation_from_matrix(T)
            eul = tf.transformations.euler_from_matrix(T)

            trans_t[1] = 0
            T = tf.TransformerROS().fromTranslationRotation(trans_t,  tf.transformations.quaternion_from_euler(
                0, eul[1], 0))

            ee_pose = self._move_group.get_current_pose().pose
            ee_pos, ee_quat = self.from_pose_msg_to_pos_quat(ee_pose)
            Tee = tf.TransformerROS().fromTranslationRotation(ee_pos, ee_quat)
            T_t = np.dot(Tee, T)
            trans_t = tf.transformations.translation_from_matrix(T_t)
            quat_t = tf.transformations.quaternion_from_matrix(T_t)

            pose_target = self.from_pos_quat_to_pose_msg(trans_t, quat_t)

            try:
                tf.TransformBroadcaster().sendTransform(trans_t,
                                                        quat_t,
                                                        rospy.Time.now(),
                                                        "mg_target",
                                                        self._move_group.get_planning_frame())
            except:
                pass

            set_target = True
        if self._target_frame != "none":
            Logger.loginfo('%s: target is  w.r.t. frame %s' %
                           (self.name, self._target_frame))
            while (True):
                if self._break_enter:
                    return
                if self._target_frame == self._end_effector_link:
                    ee_pose = self._move_group.get_current_pose().pose
                    position, quaternion = self.from_pose_msg_to_pos_quat(
                        ee_pose)
                    break

                if self._target_frame == "solar_link":
                    if self._solar_sub.has_msg('filter_solar_pose'):
                        solar_msg = self._solar_sub.get_last_msg(
                            'filter_solar_pose')
                        position, quaternion = self.from_pose_msg_to_pos_quat(
                            solar_msg.pose.pose)
                        break
                    else:
                        continue

                try:
                    t = self.tf1_lis.getLatestCommonTime(self._move_group.get_planning_frame(),
                                                         self._target_frame)
                    position, quaternion = self.tf1_lis.lookupTransform(
                        self._move_group.get_planning_frame(),
                        self._target_frame, t)
                    break
                except Exception as e:
                    Logger.logwarn('%s: exception tf1: %s' %
                                   (self.name, str(e)))
                    rospy.sleep(0.1)

            T1 = tf.TransformerROS().fromTranslationRotation(position, quaternion)
            T2 = tf.TransformerROS().fromTranslationRotation(self._pos, self._quat)
            T = np.dot(T1, T2)
            trans_t = tf.transformations.translation_from_matrix(T)
            quat_t = tf.transformations.quaternion_from_matrix(T)

            trans_t = self.do_cart_limit(trans_t)

            pose_target = self.from_pos_quat_to_pose_msg(trans_t, quat_t)
            set_target = True
        elif self._target_name != "none":
            Logger.loginfo('%s: target is srdf name: %s' %
                           (self.name, self._target_name))
            self._move_group.set_named_target(self._target_name)
        elif self._axis_num >= 0:
            Logger.loginfo('%s, target is shift' % self.name)
            self._move_group.shift_pose_target(self._axis_num, self._value)
        elif len(self._pos_targets) > 0:
            Logger.loginfo('%s: target is list' % self.name)
            set_target = True
        else:
            Logger.loginfo('%s: target is coordinate' % self.name)
            pose_target = self.from_pos_quat_to_pose_msg(
                self._pos,  self._quat)
            set_target = True

        if set_target and self._stay_level:
            current_pose = self._move_group.get_current_pose().pose
            c_pos, c_quat = self.from_pose_msg_to_pos_quat(current_pose)

            if len(self._pos_targets) > 0:
                Logger.loginfo(
                    '%s: stay level with multi waypoints' % self.name)
                targets_diff = list(
                    map(lambda p: self.pos3d_distance(c_pos, self._waypoints[p]['position']), self._pos_targets))
                self._pos_targets = self._pos_targets[targets_diff.index(
                    min(targets_diff)):]
                print('*'*10)
                print("targets_diff: ", targets_diff)

                waypoints = [current_pose]

                for p in self._pos_targets:
                    if (np.sum(np.abs(np.array(c_pos) - np.array(self._waypoints[p]['position']))) +
                            np.sum(np.abs(np.array(c_quat) - np.array(self._waypoints[p]['orientation'])))) < 0.01:
                        continue
                    temp_w = self.from_pos_quat_to_pose_msg(
                        self._waypoints[p]['position'],  self._waypoints[p]['orientation'])
                    waypoints.append(temp_w)
                    Logger.loginfo("%s: add waypoint-%s" % (self.name, p))

            else:
                Tn_t = tf.TransformerROS().fromTranslationRotation(
                    [0, 0, self._itp_norm], [0, 0, 0, 1])
                t_pos, t_quat = self.from_pose_msg_to_pos_quat(pose_target)
                Tt_b = tf.TransformerROS().fromTranslationRotation(t_pos, t_quat)
                Tn_b = np.dot(Tt_b, Tn_t)
                n_rot = tf.transformations.quaternion_from_matrix(Tn_b)
                n_trans = tf.transformations.translation_from_matrix(Tn_b)
                target_norm = self.from_pos_quat_to_pose_msg(n_trans, n_rot)

                if self._itp_norm == 0:
                    Logger.loginfo('%s: stay level with one goal' % self.name)
                    waypoints = [current_pose, pose_target]
                else:
                    Logger.loginfo(
                        '%s: stay level with one goal and norm transition' % self.name)
                    waypoints = [current_pose, target_norm, pose_target]

            jump_threshold = 0
            Logger.loginfo('%s: planning cartesian path' % self.name)
            if 0:
                constraints = self.quat_orientation_constraints(t_quat)
                self._move_group.set_path_constraints(constraints)

            plan_res = False
            for r in list(range(0, int(self._retry_num))):
                for stp in list(range(int(self._cart_step_list[0]),
                                      int(self._cart_step_list[1]))):
                    step = stp*0.1
                    (plan, fraction) = self._move_group.compute_cartesian_path(
                        waypoints, step, jump_threshold)
                    plan.joint_trajectory.points = plan.joint_trajectory.points[1:]
                    if (fraction >= 1) and self.check_plan_step(plan):
                        plan_res = True
                        Logger.loginfo("%s valid cartesian path planned, step = {0}".format(
                            step) % self.name)
                        break
                if (plan_res):
                    break

            if (not plan_res):
                Logger.logwarn("%s failed: fraction = {0}".format(
                    fraction) % self.name)
                return

            if self._if_debug:
                self.update_plan_data(plan)
                self._joint_state_sub.subscribe('joint_states', JointState,
                                                callback=self.joints_cb, buffered=False)
                self._controller_state_sub.subscribe('position_trajectory_controller/state', JointTrajectoryControllerState,
                                                     callback=self.controller_cb, buffered=False)
            if self._if_execute:
                Logger.loginfo('%s:start execution' % self.name)
                if self._move_group.execute(plan, wait=True):
                    self._res = 'done'
                    Logger.loginfo('%s succeed: executed' % self.name)
                else:
                    Logger.logwarn('%s failed: executed' % self.name)
            else:
                self._res = 'done'
            return
        elif set_target:
            self._move_group.set_pose_target(pose_target)

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

    def do_cart_limit(self, pos_raw):
        pos = list(pos_raw)
        if self._cart_limit == {}:
            return pos
        else:
            if 'x_min' in self._cart_limit:
                pos[0] = np.clip(pos[0],  self._cart_limit['x_min'], np.inf)
            if 'y_min' in self._cart_limit:
                pos[1] = np.clip(pos[1],  self._cart_limit['y_min'], np.inf)
            if 'z_min' in self._cart_limit:
                pos[2] = np.clip(pos[2],  self._cart_limit['z_min'], np.inf)
            if 'x_max' in self._cart_limit:
                pos[0] = np.clip(pos[0],   -np.inf, self._cart_limit['x_max'])
            if 'y_max' in self._cart_limit:
                pos[1] = np.clip(pos[1],   -np.inf, self._cart_limit['y_max'])
            if 'z_max' in self._cart_limit:
                # pos[2] = np.clip(pos[2],   -np.inf, self._cart_limit['z_max'])
                pos = list(self.line_interpolate(pos_raw,  np.clip(
                    pos[2],   -np.inf, self._cart_limit['z_max'])))

            return pos

    def line_interpolate(self, goal_pos, z_in):
        start_pos, _ = self.from_pose_msg_to_pos_quat(
            self._move_group.get_current_pose().pose)
        if abs(goal_pos[2] - start_pos[2]) > 0.001:
            t = (z_in - start_pos[2])/(goal_pos[2] - start_pos[2])
            x = start_pos[0] + t*(goal_pos[0] - start_pos[0])
            y = start_pos[1] + t*(goal_pos[1] - start_pos[1])
            return (x, y, z_in)
        else:
            return (goal_pos[0], goal_pos[1], z_in)

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

    def pos3d_distance(self, pos1, pos2):
        return np.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2 + (pos1[2]-pos2[2])**2)

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

    def update_plan_data(self, plan):
        self.plot_data['joints'] = plan.joint_trajectory.joint_names
        plan_points = plan.joint_trajectory.points

        self.plot_data['plan']['time'] = list(
            map(lambda p: round(p.time_from_start.to_sec(), 2), plan_points))

        for p in plan_points:
            self.plot_data['plan']['position'].append(list(p.positions))
            self.plot_data['plan']['velocities'].append(list(p.velocities))
            self.plot_data['plan']['accelerations'].append(
                list(p.accelerations))
        self.plot_data['ts'] = time.time()

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

    def from_pos_quat_to_pose_msg(self, pos, quat):
        pose_msg = Pose()
        pose_msg.position.x, pose_msg.position.y, pose_msg.position.z = pos
        pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w = quat
        return pose_msg

    def from_pose_msg_to_pos_quat(self, pose_msg):
        pos = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
        quat = [pose_msg.orientation.x, pose_msg.orientation.y,
                pose_msg.orientation.z, pose_msg.orientation.w]
        return (pos, quat)

    def quat_orientation_constraints(self, quat):
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self._reference_frame
        orientation_constraint.link_name = self._end_effector_link
        orientation_constraint.orientation.x, orientation_constraint.orientation.y, orientation_constraint.orientation.z, orientation_constraint.orientation.w = quat

        orientation_constraint.absolute_x_axis_tolerance = 2*1.57
        orientation_constraint.absolute_y_axis_tolerance = 2*1.57
        orientation_constraint.absolute_z_axis_tolerance = 2*1.57
        orientation_constraint.weight = 1.0
        constraints = Constraints()
        constraints.orientation_constraints.append(orientation_constraint)
        return constraints

    def moveit_goal_init(self):
        self._break_enter = False
        self._move_group.clear_pose_targets()
        self._move_group.set_planning_time(5)
        self._move_group.allow_replanning(True)

        self._move_group.set_max_velocity_scaling_factor(self._v_factor)
        self._move_group.set_max_acceleration_scaling_factor(self._a_factor)
        # self._move_group.set_goal_tolerance(0.01)
        # self._move_group.set_goal_orientation_tolerance(0.01)
