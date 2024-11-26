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
from comm_states.site_manipulation_ud import SiteManipulationUD
from comm_states.update_tool0_pose import CalculateNewToolPose as comm_states__CalculateNewToolPose
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Nov 15 2024
@author: hzx
'''
class test_tool0_poseSM(Behavior):
	'''
	test_not_detect
	'''


	def __init__(self):
		super(test_tool0_poseSM, self).__init__()
		self.name = 'test_tool0_pose'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:458, x:130 y:458
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.robot_pose = None
		_state_machine.userdata.tool_pose = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:40 y:82
			OperatableStateMachine.add('armInit',
										ManipulationShare(reference_frame="base_arm", end_effector_link="tool0"),
										transitions={'done': 'storetoolpose'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:190 y:286
			OperatableStateMachine.add('caltool0',
										comm_states__CalculateNewToolPose(action="pose_out", offset_x=-1.2, offset_y=0.0, offset_z=0.12),
										transitions={'done': 'movego'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group', 'robot_pose': 'robot_pose', 'tool_pose': 'tool_pose'})

			# x:46 y:355
			OperatableStateMachine.add('movego',
										SiteManipulationUD(reference_frame="base_arm", end_effector_link="tool0", v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=False, cart_step_list=[3,11], step_factor=0.1, retry_num=3, itp_norm=0.15, if_debug=False, cart_limit={}),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'mani_goal': 'tool_pose'})

			# x:38 y:165
			OperatableStateMachine.add('storetoolpose',
										comm_states__CalculateNewToolPose(action="pose_in", offset_x=-1.2, offset_y=0.0, offset_z=0.12),
										transitions={'done': 'wait2s'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group', 'robot_pose': 'robot_pose', 'tool_pose': 'tool_pose'})

			# x:54 y:255
			OperatableStateMachine.add('wait2s',
										WaitState(wait_time=2),
										transitions={'done': 'caltool0'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
