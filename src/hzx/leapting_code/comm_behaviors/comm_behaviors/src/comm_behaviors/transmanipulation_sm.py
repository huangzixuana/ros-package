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
Created on Mon Sep 18 2023
@author: ZL
'''
class TransManipulationSM(Behavior):
	'''
	trans manipulation
	'''


	def __init__(self):
		super(TransManipulationSM, self).__init__()
		self.name = 'TransManipulation'

		# parameters of this behavior
		self.add_parameter('axis', '')
		self.add_parameter('offset', 0)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:458
		_state_machine = OperatableStateMachine(outcomes=['finished'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('armInit',
										ManipulationShare(reference_frame='base_arm', end_effector_link='tool0'),
										transitions={'done': 'armManipulating'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:53 y:195
			OperatableStateMachine.add('armManipulating',
										SiteManipulation(pos=[0, 0, 0], quat=[0, 0, 0, 1], target_frame='none', target_name='none', axis_value=[self.axis, self.offset*0.01], pos_targets=[], reference_frame='base_arm', end_effector_link='tool0', v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=False, cart_step_list=[3, 11], retry_num=3, itp_norm=0.15, if_debug=False),
										transitions={'done': 'finished', 'failed': 'armManipulating'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
