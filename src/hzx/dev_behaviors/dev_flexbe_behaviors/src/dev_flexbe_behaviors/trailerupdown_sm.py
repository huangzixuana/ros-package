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
Created on Mon Apr 282024
@author: hzx
'''
class TrailerUpDownSM(Behavior):
	'''
	Trailer Up or Down
	'''


	def __init__(self):
		super(TrailerUpDownSM, self).__init__()
		self.name = 'TrailerUpDown'

		# parameters of this behavior
		self.add_parameter('height', 0.40)
		self.add_parameter('up_speed', 80)
		self.add_parameter('down_speed', 80)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:872 y:35, x:833 y:340
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:62 y:26
			OperatableStateMachine.add('lift_up',
										PublishString(name="trailer_request", value="lift_load"),
										transitions={'done': 'lift_up_speed'},
										autonomy={'done': Autonomy.Off})

			# x:523 y:28
			OperatableStateMachine.add('lift_down_speed',
										PublishString(name="trailer_request", value="{\"name\":\"lift_dump_speed\",\"lift_dump_speed\": "+str(self.down_speed)+"}"),
										transitions={'done': 'lift_goal_height'},
										autonomy={'done': Autonomy.Off})

			# x:671 y:27
			OperatableStateMachine.add('lift_goal_height',
										PublishString(name="trailer_request", value="{\"name\":\"lift_goal_height\",\"lift_goal_height\":"+str(self.height)+"}"),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:215 y:26
			OperatableStateMachine.add('lift_up_speed',
										PublishString(name="trailer_request", value="{\"name\":\"lift_load_speed\",\"lift_load_speed\": "+str(self.up_speed)+"}"),
										transitions={'done': 'lift_down'},
										autonomy={'done': Autonomy.Off})

			# x:370 y:27
			OperatableStateMachine.add('lift_down',
										PublishString(name="trailer_request", value="lift_dump"),
										transitions={'done': 'lift_down_speed'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
