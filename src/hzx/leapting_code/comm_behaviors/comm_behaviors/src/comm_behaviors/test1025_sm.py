#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_behaviors.frompick2place_sm import fromPick2PlaceSM
from comm_states.manipulation_share import ManipulationShare
from comm_states.site_manipulation import SiteManipulation
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jun 15 2023
@author: zl
'''
class test1025SM(Behavior):
	'''
	armPlaceDetectPose - armTopPose - armPickupDetectPose
	'''


	def __init__(self):
		super(test1025SM, self).__init__()
		self.name = 'test1025'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(fromPick2PlaceSM, 'fromPick2Place')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:143 y:450
		_state_machine = OperatableStateMachine(outcomes=['finished'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:104 y:62
			OperatableStateMachine.add('armInit',
										ManipulationShare(reference_frame='base_arm', end_effector_link='tool0'),
										transitions={'done': 'mid'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:94 y:261
			OperatableStateMachine.add('fromPick2Place',
										self.use_behavior(fromPick2PlaceSM, 'fromPick2Place'),
										transitions={'finished': 'finished'},
										autonomy={'finished': Autonomy.Inherit})

			# x:96 y:159
			OperatableStateMachine.add('mid',
										SiteManipulation(pos=[0, 0, 0], quat=[0, 0, 0, 1], target_frame='none', target_name='armPickupMidPose', axis_value=['none', 0], pos_targets=[], reference_frame='base_arm', end_effector_link='tool0', v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=False, cart_step_list=[3, 11], retry_num=3, itp_norm=0.15, if_debug=False),
										transitions={'done': 'fromPick2Place', 'failed': 'mid'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:360 y:445
			OperatableStateMachine.add('multi-test',
										SiteManipulation(pos=[0, 0, 0], quat=[0, 0, 0, 1], target_frame='none', target_name='none', axis_value=['none', 0], pos_targets=['armPickupMidPose','armPickupDetectPose','armTopPose','armPlaceDetectPose'], reference_frame='base_arm', end_effector_link='tool0', v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=True, cart_step_list=[5, 15], retry_num=10, itp_norm=0, if_debug=True),
										transitions={'done': 'multi-test', 'failed': 'multi-test'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
