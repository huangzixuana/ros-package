from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from geometry_msgs.msg import Pose
import numpy as np
import tf.transformations as tr

'''
Updated on Nov 11st, 2024
@author: zixuan.huang@leapting.com
'''


class CalculateNewToolPose(EventState):
    '''
    -- action            string  "pose_in" or "pose_out"
    -- offset_x          float   "pose offset_x between last and current"
    -- offset_y          float   "pose offset_y between last and current"
    -- offset_z          float   "pose offset_z between last and current"

    ># robot_pose        dict    {"pos": trans_t, "orientation": quat_t}
    ># tool_pose         dict    {"pos": trans_t, "orientation": quat_t}

    #> robot_pose        dict    {"pos": trans_t, "orientation": quat_t}
    #> tool_pose         dict    {"pos": trans_t, "orientation": quat_t}
    '''

    def __init__(self, action="pose_in", offset_x = -1.2, offset_y = 0.0, offset_z = 0.12):
        super(CalculateNewToolPose, self).__init__(
            outcomes=['done'], input_keys=['move_group', 'robot_pose', 'tool_pose'], output_keys=['robot_pose', 'tool_pose'])
        self._action = action
        self._offset_x = offset_x
        self._offset_y = offset_y
        self._offset_z = offset_z
        self._last_robot_pose = None
        self._received = False
        self._R_z = None
        self._pose_sub = ProxySubscriberCached(
            {'/robot_pose': Pose})
        self._pose_sub.subscribe("/robot_pose",Pose,
            self._robot_pose_callback)
    
    def _robot_pose_callback(self, msg):
        self._last_robot_pose = self._pose_to_dict(msg)
        self._received = True

    def on_enter(self, userdata):
        self._received = False
        theta = 3.1415926  # 180 度对应的弧度
        self._R_z = np.array([
            [np.cos(theta), -np.sin(theta), 0, 0],
            [np.sin(theta),  np.cos(theta), 0, 0],
            [0,             0,             1, 0],
            [0,             0,             0, 1]
        ])

    def execute(self, userdata):
        """Execution of the state."""
        if not self._received:
            Logger.logwarn("/robot_pose does not appear")
            return

        if self._action == "pose_in":
            current_robot_pose = self._last_robot_pose
            userdata.robot_pose = current_robot_pose
            pose_target = userdata.move_group.get_current_pose().pose
            userdata.tool_pose = self._pose_to_dict(pose_target)
            Logger.loginfo("in-tool0:({0:.5f}, {1:.5f}, {2:.5f}), ({3:.5f}, {4:.5f}, {5:.5f}, {6:.5f})"
                                    .format(userdata.tool_pose["pos"][0], userdata.tool_pose["pos"][1], 
                                            userdata.tool_pose["pos"][2], userdata.tool_pose["quat"][0],
                                            userdata.tool_pose["quat"][1], userdata.tool_pose["quat"][2],
                                            userdata.tool_pose["quat"][3]))
            Logger.loginfo("in-robot:({0:.5f}, {1:.5f}, {2:.5f}), ({3:.5f}, {4:.5f}, {5:.5f}, {6:.5f})"
                        .format(current_robot_pose["pos"][0], current_robot_pose["pos"][1], 
                                current_robot_pose["pos"][2], current_robot_pose["quat"][0],
                                current_robot_pose["quat"][1], current_robot_pose["quat"][2],
                                current_robot_pose["quat"][3]))
            return 'done'
            
        else:
            initial_robot_pose = userdata.robot_pose
            initial_tool_pose = userdata.tool_pose

            current_robot_pose = self._last_robot_pose

            # 将初始robot_pose和current_robot_pose转换为4x4齐次变换矩阵
            T_initial_robot = self._dict_to_transform(initial_robot_pose)
            T_current_robot = self._dict_to_transform(current_robot_pose)

            # 计算初始tool_pose相对于current_robot_pose的变化
            T_initial_tool = self._dict_to_transform(initial_tool_pose)
            T_initial_baselink_tool = np.dot(self._R_z, T_initial_tool)
            T_current_robot_inv = np.linalg.inv(T_current_robot)
            T_new_tool = np.dot(T_current_robot_inv, np.dot(T_initial_robot, T_initial_baselink_tool))
            T_base_arm_to_tool0 = np.dot(np.linalg.inv(self._R_z), T_new_tool)
            # 将新的tool_pose转换为字典格式
            new_tool_pose = self._transform_to_dict(T_base_arm_to_tool0)
            userdata.tool_pose = self._calcul_tool0_pose(new_tool_pose)
            Logger.loginfo("out-tool0:({0:.5f}, {1:.5f}, {2:.5f}), ({3:.5f}, {4:.5f}, {5:.5f}, {6:.5f})"
                                    .format(userdata.tool_pose["pos"][0], userdata.tool_pose["pos"][1], 
                                            userdata.tool_pose["pos"][2], userdata.tool_pose["quat"][0],
                                            userdata.tool_pose["quat"][1], userdata.tool_pose["quat"][2],
                                            userdata.tool_pose["quat"][3]))
            Logger.loginfo("out-robot:({0:.5f}, {1:.5f}, {2:.5f}), ({3:.5f}, {4:.5f}, {5:.5f}, {6:.5f})"
                        .format(current_robot_pose["pos"][0], current_robot_pose["pos"][1], 
                                current_robot_pose["pos"][2], current_robot_pose["quat"][0],
                                current_robot_pose["quat"][1], current_robot_pose["quat"][2],
                                current_robot_pose["quat"][3]))
            return 'done'

    def _pose_to_dict(self, pose_msg):
        return {
            "pos": [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z],
            "quat": [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w],
        }

    def _dict_to_transform(self, pose_dict):
        position = pose_dict["pos"]
        orientation = pose_dict["quat"]
        T = np.eye(4)
        T[:3, 3] = position
        T[:3, :3] = tr.quaternion_matrix(orientation)[:3, :3]
        return T

    def _transform_to_dict(self, T):
        position = T[:3, 3].tolist()
        orientation = tr.quaternion_from_matrix(T)
        return {
            "pos": position,
            "quat": orientation,
        }
    
    def _calcul_tool0_pose(self, pose_dict):
        position_tmp = pose_dict["pos"]
        position = [position_tmp[0] + self._offset_x, position_tmp[1] + self._offset_y, position_tmp[2] + self._offset_z]
        orientation = pose_dict["quat"]
        return {
            "pos": position,
            "quat": orientation,
        }