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
class CupOnSM(Behavior):
	'''
	suction cup on
	'''


	def __init__(self):
		super(CupOnSM, self).__init__()
		self.name = 'CupOn'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:71 y:241
		_state_machine = OperatableStateMachine(outcomes=['finished'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('suck',
										PublishJoyFeedbackArray(topic="joy_feedback_array", data_type=1, data_id=7, data_intensity=0),
										transitions={'done': 'waitSuck'},
										autonomy={'done': Autonomy.Off})

			# x:30 y:102
			OperatableStateMachine.add('waitSuck',
										WaitState(wait_time=1.5),
										transitions={'done': 'attach_pvm'},
										autonomy={'done': Autonomy.Off})

			# x:30 y:164
			OperatableStateMachine.add('attach_pvm',
										PvmManager(action='attach', pvm_size=[2.278, 1.134, 0.035], frame_id='tool0', position_z=-0.127),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
