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
class Tool0ManipulationSM(Behavior):
	'''
	tool0 manipulation
	'''


	def __init__(self):
		super(Tool0ManipulationSM, self).__init__()
		self.name = 'Tool0Manipulation'

		# parameters of this behavior
		self.add_parameter('pos_x', 0.0)
		self.add_parameter('pos_y', 0.0)
		self.add_parameter('pos_z', 0.0)
		self.add_parameter('quat_x', 0.0)
		self.add_parameter('quat_y', 0.0)
		self.add_parameter('quat_z', 0.0)
		self.add_parameter('quat_w', 1.0)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:94 y:300
		_state_machine = OperatableStateMachine(outcomes=['finished'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:86 y:57
			OperatableStateMachine.add('armInit',
										ManipulationShare(reference_frame='base_arm', end_effector_link='tool0'),
										transitions={'done': 'Tool0Manipulating'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:81 y:171
			OperatableStateMachine.add('Tool0Manipulating',
										SiteManipulation(pos=[self.pos_x, self.pos_y, self.pos_z], quat=[self.quat_x, self.quat_y, self.quat_z, self.quat_w], target_frame='tool0', target_name='none', axis_value=['none', 0], pos_targets=[], reference_frame='base_arm', end_effector_link='tool0', v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=True, cart_step_list=[1, 5], retry_num=3, itp_norm=0, if_debug=False, cart_limit={}),
										transitions={'done': 'finished', 'failed': 'Tool0Manipulating'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
