#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def publish_image():
    rospy.init_node('image_publisher', anonymous=True)
    image_pub = rospy.Publisher('image_topic', Image, queue_size=10)
    bridge = CvBridge()

    # Load your image
    img = cv2.imread('test.jpg')

    # Convert to ROS Image message
    img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")

    # Publish ROS Image message
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        image_pub.publish(img_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_image()
    except rospy.ROSInterruptException:
        pass