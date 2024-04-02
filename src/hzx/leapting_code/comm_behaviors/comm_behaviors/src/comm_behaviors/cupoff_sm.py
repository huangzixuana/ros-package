#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_states.publishjoyfeedbackarray import PublishJoyFeedbackArray
from comm_states.pvm_manager import PvmManager
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jun 21 2023
@author: ZL
'''
class CupOffSM(Behavior):
	'''
	suction cup off
	'''


	def __init__(self):
		super(CupOffSM, self).__init__()
		self.name = 'CupOff'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:42 y:336
		_state_machine = OperatableStateMachine(outcomes=['finished'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('loose',
										PublishJoyFeedbackArray(topic="joy_feedback_array", data_type=1, data_id=7, data_intensity=1),
										transitions={'done': 'waitLoose'},
										autonomy={'done': Autonomy.Off})

			# x:21 y:131
			OperatableStateMachine.add('waitLoose',
										WaitState(wait_time=3),
										transitions={'done': 'detach_pvm'},
										autonomy={'done': Autonomy.Off})

			# x:21 y:215
			OperatableStateMachine.add('detach_pvm',
										PvmManager(action='detach', pvm_size=[2.28, 1.13, 0.035], frame_id='tool0', position_z=-0.127),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
