#!/usr/bin/env python

import rospy
import tf
import json
from flexbe_core import EventState
from std_msgs.msg import String
from flexbe_core.proxy import ProxySubscriberCached
from geometry_msgs.msg import PoseWithCovarianceStamped

class ExtractModuleLocalion(EventState):
    """
    A FlexBE state to read TF relationship between /base_arm and /solar_link
    and publish JSON formatted data to /robot_command topic if values are read.
    """

    def __init__(self, action="pick"):
        """Initialization."""
        super(ExtractModuleLocalion, self).__init__(outcomes=['done'])
        self.action = action
        self.publisher = rospy.Publisher('/robot_command', String, queue_size=1)
        self._solar_msg = None
        self._received = False
        self._solar_sub = ProxySubscriberCached(
            {'filter_solar_pose': PoseWithCovarianceStamped})
        self._solar_sub.subscribe('filter_solar_pose', PoseWithCovarianceStamped,
                                  callback=self.solar_cb, buffered=False)

    def solar_cb(self, msg):
        self._solar_msg = msg
        self._received = True

    def on_enter(self, userdata):
        self._received = False

    def execute(self, userdata):
        """Execution of the state."""
        if not self._received:
            return
        
        x = self._solar_msg.pose.pose.position.x 
        y = self._solar_msg.pose.pose.position.y
        z = self._solar_msg.pose.pose.position.z
        orientation_quaternion = (
            self._solar_msg.pose.pose.orientation.x,
            self._solar_msg.pose.pose.orientation.y,
            self._solar_msg.pose.pose.orientation.z,
            self._solar_msg.pose.pose.orientation.w
        )
        rpy = tf.transformations.euler_from_quaternion(orientation_quaternion)
        roll = rpy[0]
        pitch = rpy[1]
        yaw = rpy[2]

        # Round the values to five decimal places
        x = round(x, 5)
        y = round(y, 5)
        z = round(z, 5)
        roll = round(roll, 5)
        pitch = round(pitch, 5)
        yaw = round(yaw, 5)

        # Create JSON formatted data
        data = {
            'parameter': {
                self.action+'_ideal_'+'x': x,
                self.action+'_ideal_'+'y': y,
                self.action+'_ideal_'+'z': z,
                self.action+'_ideal_'+'er': roll,
                self.action+'_ideal_'+'ep': pitch,
                self.action+'_ideal_'+'ey': yaw,
            }
        }

        # Publish JSON formatted data to /robot_command
        message = json.dumps(data)
        self.publisher.publish(message)

        return 'done'


# if __name__ == '__main__':
#     rospy.init_node('read_tf_and_publish_node')

#     state = ReadTfAndPublish()

#     outcome = state.execute(None)

#     rospy.spin()  # 这是原始代码