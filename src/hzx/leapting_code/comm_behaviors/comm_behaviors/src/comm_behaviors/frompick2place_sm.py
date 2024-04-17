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
from dev_flexbe_states.border_restrictions import BorderRestrictions as dev_flexbe_states__BorderRestrictions
from dev_flexbe_states.site_manipulation import SiteManipulation as dev_flexbe_states__SiteManipulation
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jun 15 2023
@author: zl
'''
class fromPick2PlaceSM(Behavior):
	'''
	armPickupDetectPose - armTopPose - armPlaceDetectPose
	'''


	def __init__(self):
		super(fromPick2PlaceSM, self).__init__()
		self.name = 'fromPick2Place'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:176 y:494
		_state_machine = OperatableStateMachine(outcomes=['finished'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:8 y:120
			OperatableStateMachine.add('addborder',
										dev_flexbe_states__BorderRestrictions(action="add", cube_size=[4.8,5.6,3], frame_id="base_arm", position_x=1.3, position_y=0.9, position_z=2.9),
										transitions={'done': 'armInit'},
										autonomy={'done': Autonomy.Off})

			# x:139 y:96
			OperatableStateMachine.add('armInit',
										ManipulationShare(reference_frame='base_arm', end_effector_link='tool0'),
										transitions={'done': 'multi-pick2place'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:134 y:177
			OperatableStateMachine.add('armPickupDetectPose',
										dev_flexbe_states__SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armPickupDetectPose", axis_value=["none",0], reference_frame="base_arm", end_effector_link="tool0", v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=False, cartesian_step=0.3, itp_norm=0.15, if_debug=False),
										transitions={'done': 'armTopPose', 'failed': 'armPickupDetectPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:139 y:345
			OperatableStateMachine.add('armPlaceDetectPose',
										dev_flexbe_states__SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armPlaceDetectPose", axis_value=["none",0], reference_frame="base_arm", end_effector_link="tool0", v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=False, cartesian_step=0.3, itp_norm=0.15, if_debug=False),
										transitions={'done': 'finished', 'failed': 'armPlaceDetectPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:143 y:251
			OperatableStateMachine.add('armTopPose',
										dev_flexbe_states__SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armTopPose", axis_value=["none",0], reference_frame="base_arm", end_effector_link="tool0", v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=False, cartesian_step=0.3, itp_norm=0.15, if_debug=False),
										transitions={'done': 'armPlaceDetectPose', 'failed': 'armTopPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:490 y:119
			OperatableStateMachine.add('multi-pick2place',
										dev_flexbe_states__SiteManipulation(pos=[0, 0, 0], quat=[0, 0, 0, 1], target_frame='none', target_name='none', axis_value=['none', 0], reference_frame='base_arm', end_effector_link='tool0', v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=True, cartesian_step=0.3, itp_norm=0, if_debug=True),
										transitions={'done': 'finished', 'failed': 'armPickupDetectPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
