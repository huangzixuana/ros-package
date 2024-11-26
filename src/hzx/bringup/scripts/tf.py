#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
import numpy as np
from scipy.spatial.transform import Rotation as R

def broadcast_transforms():
    rospy.init_node('tf_broadcaster', anonymous=True)
    
    # 创建 TF2 广播器
    br = tf2_ros.TransformBroadcaster()
    
    # 定时器设置时间戳
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        # livox frame 的变换信息
        livox_transform = geometry_msgs.msg.TransformStamped()
        livox_transform.header.stamp = current_time
        livox_transform.header.frame_id = "camera_color_optical_frame"
        livox_transform.child_frame_id = "livox"

        livox_transform.transform.translation.x = 0.00117584
        livox_transform.transform.translation.y = -0.0710536
        livox_transform.transform.translation.z = 0.389333

        R_matrix = np.array([[-0.00374946,    -0.999993, -0.000356629 ],
                             [-0.00575773,  0.000378214,    -0.999983],
                             [0.999976,  -0.00374734,  -0.00575911]])



        rotation = R.from_matrix(R_matrix)
        quat = rotation.as_quat()
        livox_transform.transform.rotation.x = quat[0]
        livox_transform.transform.rotation.y = quat[1]
        livox_transform.transform.rotation.z = quat[2]
        livox_transform.transform.rotation.w = quat[3]

        # livox_transform.transform.rotation.x = 0.505575
        # livox_transform.transform.rotation.y = -0.4980358
        # livox_transform.transform.rotation.z = 0.5007751
        # livox_transform.transform.rotation.w = 0.4952348

        # 发送 livox 到 base_link 的变换
        br.sendTransform(livox_transform)
        
        # link frame 的变换信息
        link_transform = geometry_msgs.msg.TransformStamped()
        link_transform.header.stamp = current_time
        link_transform.header.frame_id = "camera_color_optical_frame"
        link_transform.child_frame_id = "camera_link"

        link_transform.transform.translation.x = 0
        link_transform.transform.translation.y = 0
        link_transform.transform.translation.z = 0

        link_transform.transform.rotation.x = 0.5
        link_transform.transform.rotation.y = -0.5
        link_transform.transform.rotation.z = 0.5
        link_transform.transform.rotation.w = 0.5

        # 发送 link 到 base_link 的变换
        br.sendTransform(link_transform)

        rate.sleep()

if __name__ == '__main__':
    try:
        broadcast_transforms()
    except rospy.ROSInterruptException:
        pass