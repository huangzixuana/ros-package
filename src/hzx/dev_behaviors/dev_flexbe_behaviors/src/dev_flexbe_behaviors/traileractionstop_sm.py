#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from dev_flexbe_states.publish_string import PublishString
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Apr 28 2024
@author: hzx
'''
class TrailerActionStopSM(Behavior):
	'''
	stop the trailer updown or rotate
	'''


	def __init__(self):
		super(TrailerActionStopSM, self).__init__()
		self.name = 'TrailerActionStop'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:74 y:413, x:213 y:442
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:46 y:124
			OperatableStateMachine.add('updown_stop',
										PublishString(name="trailer_request", value="lift_stop"),
										transitions={'done': 'rotate_stop'},
										autonomy={'done': Autonomy.Off})

			# x:46 y:224
			OperatableStateMachine.add('rotate_stop',
										PublishString(name="trailer_request", value="rotate_stop"),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
