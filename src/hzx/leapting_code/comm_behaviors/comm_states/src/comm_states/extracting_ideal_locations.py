#!/usr/bin/env python

import rospy
import tf
import json
from flexbe_core import EventState
from std_msgs.msg import String

class ReadTfAndPublish(EventState):
    """
    A FlexBE state to read TF relationship between /base_arm and /solar_link
    and publish JSON formatted data to /robot_command topic if values are read.
    """

    def __init__(self, action="pick"):
        """Initialization."""
        super(ReadTfAndPublish, self).__init__(outcomes=['success', 'failed'])
        self.action = action
        self.listener = tf.TransformListener()
        self.publisher = rospy.Publisher('/robot_command', String, queue_size=1)

    def execute(self, userdata):
        """Execution of the state."""
        try:
            # Get the transformation from base_car to solar_link
            self.listener.waitForTransform('/base_arm', '/solar_link', rospy.Time(), rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform('/base_arm', '/solar_link', rospy.Time(0))

            # Extract x, y, z, roll, pitch, yaw from transformation
            x, y, z = trans
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)

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
                    'solar_'+self.action+'_x': x,
                    'solar_'+self.action+'_y': y,
                    'solar_'+self.action+'_z': z,
                    'solar_'+self.action+'_er': roll,
                    'solar_'+self.action+'_ep': pitch,
                    'solar_'+self.action+'_ey': yaw,
                }
            }

            # Publish JSON formatted data to /robot_command
            message = json.dumps(data)
            self.publisher.publish(message)

            return 'success'

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("TF lookup failed!")
            return 'failed'

# if __name__ == '__main__':
#     rospy.init_node('read_tf_and_publish_node')

#     state = ReadTfAndPublish()

#     outcome = state.execute(None)

#     rospy.spin()  # 这是原始代码