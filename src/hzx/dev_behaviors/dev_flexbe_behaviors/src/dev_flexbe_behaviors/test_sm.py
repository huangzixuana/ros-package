#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from dev_flexbe_states.border_restrictions import BorderRestrictions
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Oct 19 2023
@author: hzx
'''
class testSM(Behavior):
	'''
	test
	'''


	def __init__(self):
		super(testSM, self).__init__()
		self.name = 'test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:45 y:117
			OperatableStateMachine.add('add',
										BorderRestrictions(action="add", cube_size=[3,4.5,3.9], frame_id="base_arm", position_x=0.2, position_y=0.1, position_z=3.6),
										transitions={'done': 'remove'},
										autonomy={'done': Autonomy.High})

			# x:45 y:267
			OperatableStateMachine.add('remove',
										BorderRestrictions(action="remove", cube_size=[3,4.5,3.9], frame_id="base_arm", position_x=0.2, position_y=0.1, position_z=3.6),
										transitions={'done': 'add'},
										autonomy={'done': Autonomy.High})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
