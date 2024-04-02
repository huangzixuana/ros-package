#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_states.publisherheader import PublishHeader
from comm_states.roslaunch_node import roslaunch_node
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jun 15 2023
@author: zl
'''
class StartPVMDetectSM(Behavior):
	'''
	Start Detect PVM
	'''


	def __init__(self):
		super(StartPVMDetectSM, self).__init__()
		self.name = 'StartPVMDetect'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:641 y:61
		_state_machine = OperatableStateMachine(outcomes=['finished'], input_keys=['nav_goal'])
		_state_machine.userdata.nav_goal = {}

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:304 y:200
			OperatableStateMachine.add('startPickupSegment',
										PublishHeader(seq=2, frame_id="enable_yolov8"),
										transitions={'done': 'startPickupDetect'},
										autonomy={'done': Autonomy.Off})

			# x:571 y:191
			OperatableStateMachine.add('startPickupDetect',
										PublishHeader(seq=2, frame_id="solar_detect"),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:91 y:203
			OperatableStateMachine.add('wait4s',
										WaitState(wait_time=10),
										transitions={'done': 'startPickupSegment'},
										autonomy={'done': Autonomy.Off})

			# x:97 y:76
			OperatableStateMachine.add('launchDetect',
										roslaunch_node(cmd='roslaunch', pkg='bringup', launch_Node='apritagsam.launch'),
										transitions={'done': 'wait4s'},
										autonomy={'done': Autonomy.Off},
										remapping={'output_value': 'output_value', 'shutdown_class': 'shutdown_class'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
