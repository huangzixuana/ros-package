#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_states.bracket_obstacle import BrackeObstacle
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jun 04 2024
@author: hzx
'''
class AddBrackeobstacleSM(Behavior):
	'''
	Add Brackeobstacle
	'''


	def __init__(self):
		super(AddBrackeobstacleSM, self).__init__()
		self.name = 'AddBrackeobstacle'

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
			# x:40 y:114
			OperatableStateMachine.add('add_brackeobstacle',
										BrackeObstacle(bracke='add', obstacle='aadd', object_size=[2.278, 8.134, 0.035], frame_id='map', box_position=[0.874, 2.196, 0.758], box_orientation=[-0.016, -0.005, 0.71, 0.704]),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
