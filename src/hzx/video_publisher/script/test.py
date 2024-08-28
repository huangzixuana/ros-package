#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header

def publish_header():
    # 初始化节点
    rospy.init_node('header_publisher', anonymous=True)
    
    # 创建一个发布者
    pub = rospy.Publisher('/testhaha', Header, queue_size=10)
    
    # 设置循环频率
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        # 创建 Header 消息
        header = Header()
        header.seq = 1  # 序列号
        header.stamp = rospy.Time.now()  # 当前时间戳
        header.frame_id = 'hzx'  # 设置 frame_id 为你的名字
        
        # 发布消息
        pub.publish(header)
        rospy.loginfo("Published: %s", header)
        
        # 睡眠以保持频率
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_header()
    except rospy.ROSInterruptException:
        pass