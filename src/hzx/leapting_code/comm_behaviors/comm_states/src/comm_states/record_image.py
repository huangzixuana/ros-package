from flexbe_core import EventState, Logger
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class RecordImageState(EventState):
    """
    State: Capture a single image from a specified topic and save it as a PNG file.
    """

    def __init__(self, topic_name='/camera/image_raw', 
                 fixed_path='/home/hzx/Downloads/',
                 image_file_name='1.png'):
        """
        Initialize state, specify topic name and output path.
        """
        super(RecordImageState, self).__init__(outcomes=['done', 'failed'])
        self.topic_name = topic_name
        self.fixed_path = fixed_path
        self.output_path = os.path.join(self.fixed_path, image_file_name)
        self.bridge = CvBridge()
        self.image = None
        self.subscriber = None

    def execute(self, userdata):
        """
        Execute state: Capture an image and save it to a file.
        """
        if self.image is None:
            Logger.logerr("No image received.")
            return 'failed'

        try:
            cv2.imwrite(self.output_path, self.image)
            Logger.loginfo(f"Image saved to {self.output_path}")
            return 'done'
        except Exception as e:
            Logger.logerr(f"Error saving image: {str(e)}")
            return 'failed'

    def on_enter(self, userdata):
        """
        Actions to perform when entering the state.
        """
        Logger.loginfo(f'Subscribing to image topic: {self.topic_name}')

        # Initialize ROS node if not already initialized
        try:
            rospy.get_node_uri()  # This will raise an exception if the node is not initialized
        except rospy.ROSException:
            rospy.init_node('record_image_state_node', anonymous=True)

        # Subscribe to the image topic
        self.subscriber = rospy.Subscriber(self.topic_name, Image, self.image_callback)
        
        # Wait for image reception
        rospy.sleep(1)  # Adjust sleep time as needed

        # Ensure an image has been received
        if self.image is None:
            Logger.logerr("Failed to receive image.")
            return 'failed'

    def on_exit(self, userdata):
        """
        Actions to perform when exiting the state.
        """
        # Clean up
        if self.subscriber is not None:
            self.subscriber.unregister()
        Logger.loginfo('Unsubscribed from image topic.')

    def image_callback(self, msg):
        """
        Callback function to process received image messages.
        """
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            Logger.logerr(f"Error converting image: {str(e)}")
            self.image = None