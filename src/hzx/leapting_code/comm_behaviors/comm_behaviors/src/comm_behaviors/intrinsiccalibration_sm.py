#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_states.roslaunch_node import roslaunch_node
from comm_states.roslaunch_node_shut import roslaunch_node_shut
from comm_states.updatelaunchfile import UpdateLaunchFileState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Sep 10 2024
@author: hzx
'''
class intrinsicCalibrationSM(Behavior):
	'''
	intrinsicCalibration
	'''


	def __init__(self):
		super(intrinsicCalibrationSM, self).__init__()
		self.name = 'intrinsicCalibration'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:85 y:369, x:539 y:388
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('cameralaunch',
										roslaunch_node(cmd='roslaunch', pkg='capture_image', launch_Node='capture_image.launch'),
										transitions={'done': 'wait5s'},
										autonomy={'done': Autonomy.Off},
										remapping={'output_value': 'output_value', 'shutdown_class': 'shutdown_class1'})

			# x:52 y:159
			OperatableStateMachine.add('shutcalibration',
										roslaunch_node_shut(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'shutdown_class': 'shutdown_class2'})

			# x:259 y:160
			OperatableStateMachine.add('shutcamera',
										roslaunch_node_shut(),
										transitions={'done': 'shutcalibration'},
										autonomy={'done': Autonomy.Off},
										remapping={'shutdown_class': 'shutdown_class1'})

			# x:496 y:161
			OperatableStateMachine.add('updatelaunch',
										UpdateLaunchFileState(yaml_file_path='/home/nvidia/Downloads/calibrationdata/ost.yaml', launch_file_path='/home/nvidia/workspaces/leapting/dbparam/tmp/bringup/launch/camera.launch'),
										transitions={'success': 'shutcamera', 'fail': 'failed'},
										autonomy={'success': Autonomy.Off, 'fail': Autonomy.Off})

			# x:180 y:40
			OperatableStateMachine.add('wait5s',
										WaitState(wait_time=5),
										transitions={'done': 'calibration'},
										autonomy={'done': Autonomy.Off})

			# x:518 y:43
			OperatableStateMachine.add('waits',
										WaitState(wait_time=5),
										transitions={'done': 'updatelaunch'},
										autonomy={'done': Autonomy.Off})

			# x:345 y:42
			OperatableStateMachine.add('calibration',
										roslaunch_node(cmd='rosrun', pkg='bringup', launch_Node='auto_camera_calibrate.py'),
										transitions={'done': 'waits'},
										autonomy={'done': Autonomy.Full},
										remapping={'output_value': 'output_value', 'shutdown_class': 'shutdown_class2'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
