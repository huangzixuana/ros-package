#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_states.reconfigure_state import ReconfigureState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jul 27 2021
@author: sunkm
'''
class change_bzSM(Behavior):
	'''
	change_bz
	'''


	def __init__(self):
		super(change_bzSM, self).__init__()
		self.name = 'change_bz'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:183 y:390
		_state_machine = OperatableStateMachine(outcomes=['finished'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:146 y:174
			OperatableStateMachine.add('change_bz',
										ReconfigureState(client="move_base", parameter="base_local_planner", value="bz_local_planner/BZPlannerROS"),
										transitions={'done': 'wait2s'},
										autonomy={'done': Autonomy.Off})

			# x:157 y:74
			OperatableStateMachine.add('wait1s',
										WaitState(wait_time=1),
										transitions={'done': 'change_bz'},
										autonomy={'done': Autonomy.Off})

			# x:157 y:274
			OperatableStateMachine.add('wait2s',
										WaitState(wait_time=1),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
