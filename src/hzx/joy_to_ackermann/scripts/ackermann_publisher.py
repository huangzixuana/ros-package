#!/usr/bin/env python3

import rospy
from ackermann_msgs.msg import AckermannDrive

def ackermann_publisher():
    # 初始化 ROS 节点
    rospy.init_node('ackermann_publisher', anonymous=True)
    
    # 创建一个发布器，发布到 ackermann_drive-odom 话题
    pub = rospy.Publisher('ackermann_drive_odom', AckermannDrive, queue_size=10)
    
    # 设置消息发布的频率
    rate = rospy.Rate(10)  # 10hz
    
    steering_angle = 0.0

    while not rospy.is_shutdown():
        # 创建 AckermannDrive 消息对象
        drive_msg = AckermannDrive()
        
        # 设置消息内容
        drive_msg.speed = 1.0  # 设置速度
        drive_msg.steering_angle = steering_angle  # 设置转向角度
        
        # 发布消息
        pub.publish(drive_msg)

        steering_angle += 0.5
        
        # 按照设定的频率休眠
        rate.sleep()

if __name__ == '__main__':
    try:
        ackermann_publisher()
    except rospy.ROSInterruptException:
        pass