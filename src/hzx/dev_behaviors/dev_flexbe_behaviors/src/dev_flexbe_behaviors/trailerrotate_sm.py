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
class TrailerRotateSM(Behavior):
	'''
	trailer rotate
	'''


	def __init__(self):
		super(TrailerRotateSM, self).__init__()
		self.name = 'TrailerRotate'

		# parameters of this behavior
		self.add_parameter('goal_angle', 90)
		self.add_parameter('up_speed', 80)
		self.add_parameter('down_speed', 80)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:837 y:29, x:31 y:479
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:46 y:24
			OperatableStateMachine.add('rotate_up',
										PublishString(name="trailer_request", value="rotate_up"),
										transitions={'done': 'rotate_up_speed'},
										autonomy={'done': Autonomy.Off})

			# x:490 y:24
			OperatableStateMachine.add('rotate_down_speed',
										PublishString(name="trailer_request", value="{\"name\":\"rotate_down_speed\",\"rotate_down_speed\":"+str(self.down_speed)+"}"),
										transitions={'done': 'rotate_goal_angle'},
										autonomy={'done': Autonomy.Off})

			# x:643 y:24
			OperatableStateMachine.add('rotate_goal_angle',
										PublishString(name="trailer_request", value="{\"name\":\"rotate_goal_angle\",\"rotate_goal_angle\":"+str(self.goal_angle)+"}"),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:196 y:24
			OperatableStateMachine.add('rotate_up_speed',
										PublishString(name="trailer_request", value="{\"name\":\"rotate_up_speed\",\"rotate_up_speed\":"+str(self.up_speed)+"}"),
										transitions={'done': 'rotate_down'},
										autonomy={'done': Autonomy.Off})

			# x:346 y:24
			OperatableStateMachine.add('rotate_down',
										PublishString(name="trailer_request", value="rotate_down"),
										transitions={'done': 'rotate_down_speed'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
