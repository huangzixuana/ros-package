import yaml
import os
import shutil
import flexbe_core.logger as logger
from flexbe_core import EventState, Logger

class UpdateLaunchFileState(EventState):
    """
    A FlexBE state to update ROS launch file with camera parameters from a YAML file.
    """

    def __init__(self, yaml_file_path, launch_file_path):
        """
        Constructor for the state.
        """
        super(UpdateLaunchFileState, self).__init__(outcomes=['success', 'fail'])

        self.yaml_file_path = yaml_file_path
        self.launch_file_path = launch_file_path

    def execute(self, userdata):
        """
        Execute function of the state.
        """
        
        backup_file_path = self.launch_file_path + '.bak'
        try:
            shutil.copy(self.launch_file_path, backup_file_path)
            Logger.loginfo(f'Backup created: {backup_file_path}')
        except IOError as e:
            Logger.logerror(f'Failed to create backup: {e}')
            return 'failure'
            
        try:
            # Read YAML file
            with open(self.yaml_file_path, 'r') as yaml_file:
                yaml_data = yaml.safe_load(yaml_file)

            # Read launch file
            with open(self.launch_file_path, 'r') as launch_file:
                launch_lines = launch_file.readlines()

            # Update launch file with YAML data
            for i in range(len(launch_lines)):
                if 'D: [' in launch_lines[i]:
                    launch_lines[i] = f'            D: {yaml_data["distortion_coefficients"]["data"]}\n'
                elif 'K: [' in launch_lines[i]:
                    launch_lines[i] = f'            K: {yaml_data["camera_matrix"]["data"]}\n'
                elif 'R: [' in launch_lines[i]:
                    launch_lines[i] = f'            R: {yaml_data["rectification_matrix"]["data"]}\n'
                elif 'P: [' in launch_lines[i]:
                    launch_lines[i] = f'            P: {yaml_data["projection_matrix"]["data"]}\n'

            # Write updated content back to launch file
            with open(self.launch_file_path, 'w') as launch_file:
                launch_file.writelines(launch_lines)

            Logger.loginfo(f'Successfully updated launch file: {self.launch_file_path}')
            return 'success'

        except Exception as e:
            Logger.logerr(f'Failed to update launch file: {str(e)}')
            return 'fail'

    def on_enter(self, userdata):
        """
        Called when entering the state.
        """
        Logger.loginfo(f'Updating launch file: {self.launch_file_path}')

    def on_exit(self, userdata):
        """
        Called when exiting the state.
        """
        Logger.loginfo('Exiting UpdateLaunchFileState.')


