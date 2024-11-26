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
Created on Thu Jun 15 2023
@author: zl
'''
class fromPlace2PickSM(Behavior):
	'''
	armPlaceDetectPose - armTopPose - armPickupDetectPose
	'''


	def __init__(self):
		super(fromPlace2PickSM, self).__init__()
		self.name = 'fromPlace2Pick'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:381 y:347
		_state_machine = OperatableStateMachine(outcomes=['finished'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:104 y:62
			OperatableStateMachine.add('armInit',
										ManipulationShare(reference_frame='base_arm', end_effector_link='tool0'),
										transitions={'done': 'armPlaceDetectPose'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:98 y:340
			OperatableStateMachine.add('armPickupDetectPose',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armPickupDetectPose", axis_value=["none",0], pos_targets=[], trajectory_name="none", reference_frame="base_arm", end_effector_link="tool0", wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=False, cart_step_list=[3, 11], step_factor=0.1, itp_norm=0.15, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id='none', plan_time=2),
										transitions={'done': 'finished', 'failed': 'armPickupDetectPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:99 y:154
			OperatableStateMachine.add('armPlaceDetectPose',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armPlaceDetectPose", axis_value=["none",0], pos_targets=[], trajectory_name="none", reference_frame="base_arm", end_effector_link="tool0", wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=False, cart_step_list=[3, 11], step_factor=0.1, itp_norm=0.15, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id='none', plan_time=2),
										transitions={'done': 'armPickupDetectPose', 'failed': 'armPlaceDetectPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:209 y:231
			OperatableStateMachine.add('armTopPose',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armTopPose", axis_value=["none",0], pos_targets=[], trajectory_name="none", reference_frame="base_arm", end_effector_link="tool0", wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=False, cart_step_list=[3, 11], step_factor=0.1, itp_norm=0.15, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id='none', plan_time=2),
										transitions={'done': 'armPickupDetectPose', 'failed': 'armTopPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:343 y:61
			OperatableStateMachine.add('multi-place2pick',
										SiteManipulation(pos=[0, 0, 0], quat=[0, 0, 0, 1], target_frame='none', target_name='none', axis_value=['none', 0], pos_targets=['armPlaceDetectPose','armTopPose','armPickupDetectPose'], trajectory_name="none", reference_frame='base_arm', end_effector_link='tool0', wait_time=0.5, v_factor=1, a_factor=1, t_factor=1.0, stay_level=True, cart_step_list=[1, 10], step_factor=0.1, itp_norm=0, retry_num=3, cart_limit={}, if_execute=True, if_debug=True, planner_id='none', plan_time=2),
										transitions={'done': 'finished', 'failed': 'armPlaceDetectPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
