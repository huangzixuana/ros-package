#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy

def main():
  # 初始化 ROS 节点和发布者
  rospy.init_node('joy_publisher')
  pub = rospy.Publisher('/joy', Joy, queue_size=1)

  # 创建一个sensor_msgs/Joy消息对象
  joy_msg = Joy()

  # 设置消息数据
  joy_msg.header.stamp = rospy.Time.now()
  joy_msg.header.frame_id = "/can0"
  joy_msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0,0]
  joy_msg.buttons = [0, 0, 0, 0, 0, 0,0,0,0,0,0]

  rate = rospy.Rate(10) # 发布频率为10Hz

  while not rospy.is_shutdown():
    # 设置轴和按钮的值
    joy_msg.axes[1] = 0 # 假设左右摇杆 y 轴处于负方向
    joy_msg.axes[3] = -0.2
    joy_msg.axes[2] = 0.5
    joy_msg.axes[5] = 0.0
    joy_msg.buttons[6] = 0 # 假设按钮1已经按下
    joy_msg.buttons[0] = 1

    # 发布消息
    pub.publish(joy_msg)

    rate.sleep()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
