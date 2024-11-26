from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
import subprocess
import time
import os

class RecordBagState(EventState):
    """
    State to record a specific topic to a bag file.
    """

    def __init__(self, topic_name='/livox/lidar', 
                 record_duration=15,
                 fixed_path='/home/hzx/Downloads/',
                 bag_file_name='1.bag'):
        """
        Initialize state with topic name, recording duration, and fixed path.
        """
        super(RecordBagState, self).__init__(outcomes=['done', 'failed'])
        self.topic_name = topic_name
        self.record_duration = record_duration
        self.fixed_path = fixed_path
        self.bag_file_name = os.path.join(self.fixed_path, bag_file_name)
        self.process = None

    def execute(self, userdata):
        """
        Execute the state: start recording and wait until recording is complete.
        """
        if self.process:
            self.process.terminate()

        # Ensure directory exists
        if not os.path.exists(self.fixed_path):
            try:
                os.makedirs(self.fixed_path)
            except Exception as e:
                Logger.logerr(f"Failed to create directory: {str(e)}")
                return 'failed'

        # Start recording
        try:
            self.process = subprocess.Popen(['rosbag', 'record', '-O', self.bag_file_name, self.topic_name])
            time.sleep(self.record_duration)
            self.process.terminate()  # Stop recording
            return 'done'
        except Exception as e:
            Logger.logerr(f"Error during recording: {str(e)}")
            return 'failed'

    def on_enter(self, userdata):
        """
        Actions to perform when entering the state.
        """
        Logger.loginfo(f'Starting recording topic data to {self.bag_file_name}...')

    def on_exit(self, userdata):
        """
        Actions to perform when exiting the state.
        """
        if self.process:
            self.process.terminate()
            Logger.loginfo('Recording stopped.')

# Usage example:
# state = RecordBagState(topic_name='/livox/lidar', record_duration=15, fixed_path='/path/to/your/directory/')
