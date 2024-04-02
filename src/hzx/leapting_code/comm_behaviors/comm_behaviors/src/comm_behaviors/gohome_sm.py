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
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jun 07 2023
@author: why
'''
class GoHomeSM(Behavior):
	'''
	GoHome
	'''


	def __init__(self):
		super(GoHomeSM, self).__init__()
		self.name = 'GoHome'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:275, x:130 y:275
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.move_group = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:268 y:22
			OperatableStateMachine.add('armTopPose',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armTopPose", axis_value=["none",0], pos_targets=[], reference_frame="base_arm", end_effector_link="tool0", v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=False, cart_step_list=[3, 11], retry_num=3, itp_norm=0.15, if_debug=False),
										transitions={'done': 'finished', 'failed': 'armTopPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:273 y:146
			OperatableStateMachine.add('armInitPose',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armInitPose", axis_value=["none",0], pos_targets=[], reference_frame="base_arm", end_effector_link="tool0", v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=False, cart_step_list=[3, 11], retry_num=3, itp_norm=0.15, if_debug=False),
										transitions={'done': 'finished', 'failed': 'armInitPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
