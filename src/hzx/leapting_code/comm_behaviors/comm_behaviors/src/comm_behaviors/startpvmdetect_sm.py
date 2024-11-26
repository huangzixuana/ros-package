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
		# x:640 y:63
		_state_machine = OperatableStateMachine(outcomes=['finished'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:122 y:60
			OperatableStateMachine.add('startPickupSegment',
										PublishHeader(seq=2, frame_id="enable_yolov8"),
										transitions={'done': 'startPickupDetect'},
										autonomy={'done': Autonomy.Off})

			# x:363 y:59
			OperatableStateMachine.add('startPickupDetect',
										PublishHeader(seq=2, frame_id="solar_detect"),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
