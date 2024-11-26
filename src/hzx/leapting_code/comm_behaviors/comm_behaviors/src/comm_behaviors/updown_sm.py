#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_states.manipulation_share import ManipulationShare
from comm_states.site_manipulation import SiteManipulation
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jan 30 2024
@author: zcx
'''
class UpDownSM(Behavior):
	'''
	test
	'''


	def __init__(self):
		super(UpDownSM, self).__init__()
		self.name = 'UpDown'

		# parameters of this behavior
		self.add_parameter('height', 0.0)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:833 y:31
		_state_machine = OperatableStateMachine(outcomes=['done'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:109 y:26
			OperatableStateMachine.add('armInit',
										ManipulationShare(reference_frame="base_arm", end_effector_link="tool0"),
										transitions={'done': 'go'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:435 y:26
			OperatableStateMachine.add('go',
										SiteManipulation(pos=[0, 0, self.height], quat=[0,0,0,1], target_frame='tool0', target_name='none', axis_value=['none', 0], pos_targets=[], trajectory_name='none', reference_frame='base_arm', end_effector_link='tool0', wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=True, cart_step_list=[3, 11], step_factor=0.1, itp_norm=0, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id='none', plan_time=2),
										transitions={'done': 'done', 'failed': 'go'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
