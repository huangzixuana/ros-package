#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from dev_flexbe_states.border_restrictions import BorderRestrictions as dev_flexbe_states__BorderRestrictions
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Oct 19 2023
@author: hzx
'''
class add_border_restrictionsSM(Behavior):
	'''
	add_border_restrictions
	'''


	def __init__(self):
		super(add_border_restrictionsSM, self).__init__()
		self.name = 'add_border_restrictions'

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
			# x:126 y:136
			OperatableStateMachine.add('add_border_restrictions',
										dev_flexbe_states__BorderRestrictions(action="add", cube_size=[5.8,6.6,4], frame_id="base_arm", position_x=2, position_y=0.9, position_z=3.2),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
