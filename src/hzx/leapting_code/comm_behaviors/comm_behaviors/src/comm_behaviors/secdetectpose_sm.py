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
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Nov 12 2024
@author: hzx
'''
class secDetectPoseSM(Behavior):
	'''
	let arm to secDetectPose
	'''


	def __init__(self):
		super(secDetectPoseSM, self).__init__()
		self.name = 'secDetectPose'

		# parameters of this behavior
		self.add_parameter('sec_offset_x', 0.0)
		self.add_parameter('sec_offset_y', 0.0)
		self.add_parameter('sec_offset_z', 0.0)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:458
		_state_machine = OperatableStateMachine(outcomes=['finished'], input_keys=['robot_pose', 'tool_pose'], output_keys=['robot_pose', 'tool_pose'])
		_state_machine.userdata.robot_pose = None
		_state_machine.userdata.tool_pose = None
		_state_machine.userdata.move_group = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:66 y:72
			OperatableStateMachine.add('arm_init',
										ManipulationShare(reference_frame="base_arm", end_effector_link="tool0"),
										transitions={'done': 'calculationTool0Pose'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:62 y:198
			OperatableStateMachine.add('calculationTool0Pose',
										comm_states__CalculateNewToolPose(action="pose_out", offset_x=-1.2 + self.sec_offset_x, offset_y=0.0 + self.sec_offset_y, offset_z=0.12 + self.sec_offset_z),
										transitions={'done': 'secDetectPose'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group', 'robot_pose': 'robot_pose', 'tool_pose': 'tool_pose'})

			# x:69 y:314
			OperatableStateMachine.add('secDetectPose',
										SiteManipulationUD(reference_frame="base_arm", end_effector_link="tool0", v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=True, cart_step_list=[3,11], step_factor=0.1, retry_num=3, itp_norm=0.15, if_debug=False, cart_limit={}),
										transitions={'done': 'finished', 'failed': 'secDetectPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'mani_goal': 'tool_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
