#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_states.site_manipulation import SiteManipulation
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Aug 25 2023
@author: ZL
'''
class testarmSM(Behavior):
	'''
	test-arm
	'''


	def __init__(self):
		super(testarmSM, self).__init__()
		self.name = 'test-arm'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:163 y:567
		_state_machine = OperatableStateMachine(outcomes=['finished'])
		_state_machine.userdata.nav_goal = {}
		_state_machine.userdata.move_group = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:165 y:200
			OperatableStateMachine.add('arm',
										SiteManipulation(pos=[0, 0, 1.6], quat=[0, 0, 0, 1], target_frame='none', target_name='calibration_18', axis_value=['none', 0], pos_targets=[], reference_frame='base_arm', end_effector_link='tool0', v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=True, cart_step_list=[3, 11], retry_num=3, itp_norm=0, if_debug=False, cart_limit={'z_max':0.5}),
										transitions={'done': 'finished', 'failed': 'wait'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:332 y:232
			OperatableStateMachine.add('wait',
										WaitState(wait_time=1),
										transitions={'done': 'arm'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
