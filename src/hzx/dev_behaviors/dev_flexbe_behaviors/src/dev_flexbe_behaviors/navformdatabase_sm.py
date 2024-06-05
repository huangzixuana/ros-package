#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from dev_flexbe_states.g21_trig import g21_trig
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat May 11 2024
@author: hzx
'''
class NavFormDatabaseSM(Behavior):
	'''
	Read Nac_pose from  database
	'''


	def __init__(self):
		super(NavFormDatabaseSM, self).__init__()
		self.name = 'NavFormDatabase'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:83 y:340, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:46 y:74
			OperatableStateMachine.add('request',
										g21_trig(seq=10, frame_id="execute_mission"),
										transitions={'done': 'wait3s'},
										autonomy={'done': Autonomy.Off})

			# x:210 y:122
			OperatableStateMachine.add('wait3s',
										WaitState(wait_time=3),
										transitions={'done': 'point1'},
										autonomy={'done': Autonomy.Off})

			# x:46 y:174
			OperatableStateMachine.add('point1',
										g21_trig(seq=25, frame_id="request_object"),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
