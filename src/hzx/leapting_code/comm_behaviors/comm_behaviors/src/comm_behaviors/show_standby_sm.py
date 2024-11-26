#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_behaviors.cupon_sm import CupOnSM
from comm_states.manipulation_share import ManipulationShare
from comm_states.site_manipulation import SiteManipulation
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jun 11 2024
@author: ZL
'''
class showstandbySM(Behavior):
	'''
	test
	'''


	def __init__(self):
		super(showstandbySM, self).__init__()
		self.name = 'show-standby'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(CupOnSM, 'CupOn')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:749 y:591
		_state_machine = OperatableStateMachine(outcomes=['finished'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:99 y:174
			OperatableStateMachine.add('armInit',
										ManipulationShare(reference_frame='base_arm', end_effector_link='tool0'),
										transitions={'done': 'CupOn'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:379 y:207
			OperatableStateMachine.add('armPickup-todo',
										SiteManipulation(pos=[0, 0, 0], quat=[0, 0, 0, 1], target_frame='none', target_name='armPickDetect', axis_value=['none', 0], pos_targets=[], trajectory_name='none', reference_frame='base_arm', end_effector_link='tool0', wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=False, cart_step_list=[3, 11], step_factor=0.1, itp_norm=0.15, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id='none', plan_time=2),
										transitions={'done': 'wait2s', 'failed': 'armPickup-todo'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:368 y:112
			OperatableStateMachine.add('armPickupDetectPose',
										SiteManipulation(pos=[0, 0, 0], quat=[0, 0, 0, 1], target_frame='none', target_name='armPickupDetectPose', axis_value=['none', 0], pos_targets=[], trajectory_name='none', reference_frame='base_arm', end_effector_link='tool0', wait_time=2, v_factor=1, a_factor=1, t_factor=1.0, stay_level=False, cart_step_list=[3, 11], step_factor=0.1, itp_norm=0.15, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id='none', plan_time=2),
										transitions={'done': 'armPickup-todo', 'failed': 'armPickupDetectPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:676 y:237
			OperatableStateMachine.add('armPlace',
										SiteManipulation(pos=[-0.03564049, 2.30242423, 0.52617598], quat=[0.01145503, 0.01130756,0.71698644,0.69690126], target_frame='none', target_name='none', axis_value=['none', 0], pos_targets=[], trajectory_name='none', reference_frame='base_arm', end_effector_link='tool0', wait_time=2, v_factor=1, a_factor=1, t_factor=1.0, stay_level=True, cart_step_list=[3, 11], step_factor=0.1, itp_norm=0.15, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id='none', plan_time=2),
										transitions={'done': 'up-40cm', 'failed': 'armPlace'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:665 y:383
			OperatableStateMachine.add('armPlaceDetectPose',
										SiteManipulation(pos=[0, 0, 0], quat=[0, 0, 0, 1], target_frame='none', target_name='armPlaceDetectPose', axis_value=['none', 0], pos_targets=[], trajectory_name='none', reference_frame='base_arm', end_effector_link='tool0', wait_time=2, v_factor=1, a_factor=1, t_factor=1.0, stay_level=False, cart_step_list=[3, 11], step_factor=0.1, itp_norm=0.15, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id='none', plan_time=2),
										transitions={'done': 'armPlace', 'failed': 'armPlaceDetectPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:675 y:114
			OperatableStateMachine.add('up-40cm',
										SiteManipulation(pos=[0, 0, 0.4], quat=[0, 0, 0, 1], target_frame='tool0', target_name='none', axis_value=['none', 0], pos_targets=[], trajectory_name='none', reference_frame='base_arm', end_effector_link='tool0', wait_time=0.5, v_factor=1, a_factor=1, t_factor=1.0, stay_level=False, cart_step_list=[1, 11], step_factor=0.1, itp_norm=0, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id='none', plan_time=2),
										transitions={'done': 'armPickupDetectPose', 'failed': 'up-40cm'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:377 y:386
			OperatableStateMachine.add('up-50cm',
										SiteManipulation(pos=[0, 0, 0.5], quat=[0, 0, 0, 1], target_frame='tool0', target_name='none', axis_value=['none', 0], pos_targets=[], trajectory_name='none', reference_frame='base_arm', end_effector_link='tool0', wait_time=0.5, v_factor=1, a_factor=1, t_factor=1.0, stay_level=False, cart_step_list=[1, 11], step_factor=0.1, itp_norm=0.0, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id='none', plan_time=2),
										transitions={'done': 'armPlaceDetectPose', 'failed': 'up-50cm'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:384 y:300
			OperatableStateMachine.add('wait2s',
										WaitState(wait_time=2),
										transitions={'done': 'up-50cm'},
										autonomy={'done': Autonomy.Off})

			# x:89 y:314
			OperatableStateMachine.add('CupOn',
										self.use_behavior(CupOnSM, 'CupOn'),
										transitions={'finished': 'armPickupDetectPose'},
										autonomy={'finished': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
