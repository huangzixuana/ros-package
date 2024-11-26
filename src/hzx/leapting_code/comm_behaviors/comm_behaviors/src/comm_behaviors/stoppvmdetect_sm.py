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
class StopPVMDetectSM(Behavior):
	'''
	Stop Detect PVM
	'''


	def __init__(self):
		super(StopPVMDetectSM, self).__init__()
		self.name = 'StopPVMDetect'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:643 y:85
		_state_machine = OperatableStateMachine(outcomes=['finished'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:173 y:80
			OperatableStateMachine.add('stopPickupSegment',
										PublishHeader(seq=0, frame_id="enable_yolov8"),
										transitions={'done': 'stopPickupDetect'},
										autonomy={'done': Autonomy.Off})

			# x:372 y:80
			OperatableStateMachine.add('stopPickupDetect',
										PublishHeader(seq=0, frame_id="solar_detect"),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
