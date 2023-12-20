#!/usr/bin/env python

from flexbe_core import EventState, Logger
import rospy
import sys
import copy
import moveit_commander
import tf
import time
import tf2_ros
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

'''
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
                 reference_frame="base_arm",
                 end_effector_link="tool0",
                 v_factor=1,
                 a_factor=1,
                 if_execute=True,
                 wait_time=0,
                 stay_level=False,
                 cartesian_step=0.3,
                 itp_norm=0.15,
                 if_debug=False):
        super(SiteManipulation, self).__init__(outcomes=['done', 'failed'])
        axis_dict = {'X': 0, 'Y': 1, 'Z': 2,
                     'R': 3, 'P': 4, 'YAW': 5, 'none': -1}
        self._pos, self._quat = pos, quat
        self._target_frame, self._target_name = target_frame, target_name
        self._v_factor, self._a_factor = v_factor, a_factor
        self._axis_num, self._value = axis_dict[axis_value[0]], axis_value[1]

        self._if_execute = if_execute
        self.stay_level = stay_level
        self._wait_time = wait_time
        self._cartesian_step = cartesian_step
        self._itp_norm = itp_norm
        self._if_debug = if_debug
        self._res = ''

        group_name = rospy.get_param(
            "/move_group/moveit_sim_hw_interface/joint_model_group", "leapting_arm")
        # self._cancel_pub = ProxyPublisher(
        #     {'position_trajectory_controller/follow_joint_trajectory/cancel': GoalID})
        self._traj_pub = ProxyPublisher(
            {'position_trajectory_controller/command': JointTrajectory})

        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        self._move_group = moveit_commander.MoveGroupCommander(group_name)
        if reference_frame != "":
            self._move_group.set_pose_reference_frame(reference_frame)
        if end_effector_link != "":
            self._move_group.set_end_effector_link(end_effector_link)
        self.tf1_lis = TransformListener()
        self.entered = False
        self._preempted = False

        self._pause_sub = ProxySubscriberCached({'flexbe/command/pause': Bool})
        self._pause_sub.set_callback('flexbe/command/pause', self.pause_cb)
        self._preempt_sub = ProxySubscriberCached(
            {'flexbe/command/preempt': Empty})
        self._preempt_sub.set_callback(
            'flexbe/command/preempt', self.preempt_cb)
        self._br_target = tf.TransformBroadcaster()

        if self._if_debug:
            self.init_data()
            self._joint_state_sub = ProxySubscriberCached(
                {'joint_states': JointState})
            self._controller_state_sub = ProxySubscriberCached(
                {'position_trajectory_controller/state': JointTrajectoryControllerState})
            if not os.path.isfile(self.plot_data['path']):
                with open(self.plot_data['path'], 'w') as fp:
                    pass

    def on_enter(self, userdata):
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
                    t = self.tf1_lis.getLatestCommonTime(self._move_group.get_end_effector_link(),
                                                         self._target_frame)
                    position, quaternion = self.tf1_lis.lookupTransform(
                        self._move_group.get_end_effector_link(),
                        self._target_frame, t)
                    break
                except Exception as e:
                    Logger.logwarn('%s: exception tf1: %s' %
                                   (self.name, str(e)))
                    rospy.sleep(1)

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
            # self._move_group.set_pose_target(pose_target)
            try:
                self._br_target.sendTransform(trans_t,
                                              quat_t,
                                              rospy.Time.now(),
                                              "mg_target",
                                              self._move_group.get_planning_frame())
            except:
                pass

            set_target = True
        if self._target_frame is not "none":

            # tf_buf = tf2_ros.Buffer()
            # tf_listener = tf2_ros.TransformListener(tf_buf)
            # tf2_ros.Buffer.clear(tf_buf)
            Logger.loginfo('%s: target is  w.r.t. frame %s' %
                           (self.name, self._target_frame))

            while (True):
                # try:
                #     tform = tf_buf.lookup_transform(
                #         self._move_group.get_planning_frame(),
                #         self._target_frame,
                #         rospy.Time()).transform
                #     trans, rot = tform.translation, tform.rotation
                #     position = [trans.x, trans.y, trans.z]
                #     quaternion = [rot.x, rot.y, rot.z, rot.w]
                #     break
                # except Exception as e:
                #     Logger.logwarn('exception tf2: %s' % str(e))

                #     # tf2_ros.Buffer.clear(tf_buf)
                #     # tf_buf = tf2_ros.Buffer()
                #     # tf_listener = tf2_ros.TransformListener(tf_buf)
                #     rospy.sleep(1)

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
                    rospy.sleep(1)

            T1 = tf.TransformerROS().fromTranslationRotation(position, quaternion)
            T2 = tf.TransformerROS().fromTranslationRotation(self._pos, self._quat)
            T = np.dot(T1, T2)
            trans_t = tf.transformations.translation_from_matrix(T)
            quat_t = tf.transformations.quaternion_from_matrix(T)
            pose_target = self.from_pos_quat_to_pose_msg(trans_t, quat_t)
            # self._move_group.set_pose_target(pose_target)
            set_target = True
        elif self._target_name is not "none":
            Logger.loginfo('%s: target is srdf name: %s' %
                           (self.name, self._target_name))
            self._move_group.set_named_target(self._target_name)
        elif self._axis_num >= 0:
            Logger.loginfo('%s, target is shift' % self.name)
            self._move_group.shift_pose_target(self._axis_num, self._value)

            '''
            trans = [0, 0, 0]
            rot = [0, 0, 0, 1]
            eul = [0, 0, 0]
            if self._axis_num <= 2:
                trans[self._axis_num] = self._value
            else:
                eul[self._axis_num-3] = self._value
            rot = tf.transformations.quaternion_from_euler(
                eul[0], eul[1], eul[2])
            To_ee = tf.TransformerROS().fromTranslationRotation(trans, rot)
            pos_t, quat_t = self.from_pose_msg_to_pos_quat(pose_target)
            Tee_base = tf.TransformerROS().fromTranslationRotation(pos_t, quat_t)
            To_base = np.dot(Tee_base, To_ee)
            rot_target = tf.transformations.quaternion_from_matrix(To_base)

            trans_target = tf.transformations.translation_from_matrix(To_base)
            pose_target = self.from_pos_quat_to_pose_msg(
                trans_target, rot_target)
            self._move_group.set_pose_target(pose_target)
            '''
        else:
            Logger.loginfo('%s: target is coordinate' % self.name)
            pose_target = self.from_pos_quat_to_pose_msg(
                self._pos,  self._quat)
            # self._move_group.set_pose_target(pose_target)
            set_target = True

        if set_target and self.stay_level:
            Logger.loginfo('%s: ee stay level' % self.name)

            Tn_t = tf.TransformerROS().fromTranslationRotation(
                [0, 0, self._itp_norm], [0, 0, 0, 1])
            t_pos, t_quat = self.from_pose_msg_to_pos_quat(pose_target)
            Tt_b = tf.TransformerROS().fromTranslationRotation(t_pos, t_quat)
            Tn_b = np.dot(Tt_b, Tn_t)
            n_rot = tf.transformations.quaternion_from_matrix(Tn_b)
            n_trans = tf.transformations.translation_from_matrix(Tn_b)
            target_norm = self.from_pos_quat_to_pose_msg(n_trans, n_rot)
            current_pose = self._move_group.get_current_pose().pose
            if self._itp_norm == 0:
                waypoints = [current_pose, pose_target]
            else:
                waypoints = [current_pose, target_norm, pose_target]

            (step, jump_threshold) = (self._cartesian_step, 0.00)
            Logger.loginfo('%s: planning cartesian path' % self.name)
            if 0:
                constraints = self.quat_orientation_constraints(t_quat)
                self._move_group.set_path_constraints(constraints)
            (plan, fraction) = self._move_group.compute_cartesian_path(
                waypoints, step, jump_threshold)

            if len(plan.joint_trajectory.points) < 2:
                Logger.loginfo('%s failed: path points too few' % self.name)
                return
            elif fraction < 1:
                Logger.logwarn("%s failed: fraction = {0}".format(
                    fraction) % self.name)
                return

            Logger.loginfo("%s: valid cartesian path planned" % self.name)
            plan.joint_trajectory.points = plan.joint_trajectory.points[1:]

            ps = copy.deepcopy(plan.joint_trajectory.points)
            # if len(ps)>10:
            if 0:
                time_list = [ps[0].time_from_start.to_sec()]
                for i in range(len(ps)-1):
                    dt = (ps[i+1].time_from_start.to_sec() -
                          ps[i].time_from_start.to_sec())
                    if i > len(ps)*0.7:
                        end_f = 1.3*round(1.0/self._v_factor, 2)
                    else:
                        end_f = 1.0*round(1.0/self._v_factor, 2)
                    time_list.append(dt*end_f)
                for i in range(len(ps)):
                    plan.joint_trajectory.points[i].time_from_start = rospy.Duration(
                        sum(time_list[:i+1]))

            if self._if_debug:
                self.update_plan_data(plan)
                self._joint_state_sub.subscribe('joint_states', JointState,
                                                callback=self.joints_cb, buffered=False)
                self._controller_state_sub.subscribe('position_trajectory_controller/state', JointTrajectoryControllerState,
                                                     callback=self.controller_cb, buffered=False)
            if self._if_execute:
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
            if plan_msg[0]:
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
                Logger.logwarn("%s:  no path" % self.name)

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
            # for j in range(len(pos)):
            #     self.plot_data['real']['position'][j].append(
            #         [round(pos[j], 3)])
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

            # if self.joint_plan_diff(pos, 0.5):
            #     print(pos)
            #     Logger.logerr(
            #         'joint executed deviation exceeds the threshold 0.5')
        else:
            pass

    def joint_joint_diff(self, pos1, pos2, d=0.05):
        for j in range(len(pos1)):
            if abs(pos1[j] - pos2[j]) > d:
                return True
        return False

    def joint_plan_diff(self, pos1, d=0.1):
        if self.plot_data['plan']['position'] == []:
            rospy.logwarn('plan is empty')
            return False

        # for i in range(len(self.plot_data['plan']['position'][0])):
        #     pos2 = [row[i] for row in self.plot_data['plan']['position']]
        #     if not self.joint_joint_diff(pos1, pos2, d):
        #         return False

        for p in self.plot_data['plan']['position']:
            if not self.joint_joint_diff(pos1, p, d):
                return False
        return True

    def update_plan_data(self, plan):
        self.plot_data['joints'] = plan.joint_trajectory.joint_names
        plan_points = plan.joint_trajectory.points

        self.plot_data['plan']['time'] = list(
            map(lambda p: round(p.time_from_start.to_sec(), 2), plan_points))

        # for j in range(len(plan.joint_trajectory.joint_names)):
        #     temp_list = list(
        #         map(lambda p: round(p.positions[j], 3), plan_points))

        #     self.plot_data['plan']['position'].append(temp_list)
        #     self.plot_data['real']['position'].append([])

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
        orientation_constraint.header.frame_id = 'base_arm'
        orientation_constraint.link_name = 'tool0'
        orientation_constraint.orientation.x, orientation_constraint.orientation.y, orientation_constraint.orientation.z, orientation_constraint.orientation.w = quat

        orientation_constraint.absolute_x_axis_tolerance = 2*1.57
        orientation_constraint.absolute_y_axis_tolerance = 2*1.57
        orientation_constraint.absolute_z_axis_tolerance = 2*1.57
        orientation_constraint.weight = 1.0
        constraints = Constraints()
        constraints.orientation_constraints.append(orientation_constraint)
        return constraints

    def moveit_goal_init(self):
        self._move_group.clear_pose_targets()
        self._move_group.set_planning_time(30)
        # self._move_group.set_num_planning_attempts(15)
        self._move_group.allow_replanning(True)

        self._move_group.set_max_velocity_scaling_factor(self._v_factor)
        self._move_group.set_max_acceleration_scaling_factor(self._a_factor)
        # self._move_group.set_goal_tolerance(0.01)
        # self._move_group.set_goal_orientation_tolerance(0.01)
