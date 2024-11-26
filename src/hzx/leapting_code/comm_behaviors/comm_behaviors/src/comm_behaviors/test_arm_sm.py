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
Created on Thu Jun 06 2024
@author: test
'''
class testarmSM(Behavior):
	'''
	test
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
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('armInit',
										ManipulationShare(reference_frame='base_arm', end_effector_link='tool0'),
										transitions={'done': 'armTest'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:32 y:159
			OperatableStateMachine.add('armTest',
										SiteManipulation(pos=[0, 0, 0.3], quat=[-0.7071, 0, 0, 0.7071], target_frame='bundle_67', target_name='none', axis_value=['none', 0], pos_targets=[], trajectory_name='none', reference_frame='base_arm', end_effector_link='tool0', wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=True, cart_step_list=[1, 18], step_factor=0.1, itp_norm=0.0, retry_num=3, cart_limit={}, if_execute=True, if_debug=True, planner_id='none', plan_time=2),
										transitions={'done': 'finished', 'failed': 'armTest'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:291 y:170
			OperatableStateMachine.add('x55cm',
										SiteManipulation(pos=[0.0091,0.0039,0], quat=[0,0,0,1], target_frame="tool0", target_name="none", axis_value=['none', 0], pos_targets=[], trajectory_name="none", reference_frame="base_arm", end_effector_link="tool0", wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=True, cart_step_list=[1,11], step_factor=0.01, itp_norm=0, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id="none", plan_time=2),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
