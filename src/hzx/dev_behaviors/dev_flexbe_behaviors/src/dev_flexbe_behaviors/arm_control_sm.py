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
from dev_flexbe_states.scene_manager import SceneManager
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jun 14 2024
@author: hzx
'''
class arm_controlSM(Behavior):
	'''
	test the arm control
	'''


	def __init__(self):
		super(arm_controlSM, self).__init__()
		self.name = 'arm_control'

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
										ManipulationShare(reference_frame="base_arm", end_effector_link="tool0"),
										transitions={'done': 'armPickupDetectPose'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:24 y:124
			OperatableStateMachine.add('armPickupDetectPose',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armPickupDetectPose", axis_value=["none",0], pos_targets=[], reference_frame="base_arm", end_effector_link="tool0", v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=False, cart_step_list=[3,11], retry_num=3, itp_norm=0.15, if_debug=False, cart_limit={}),
										transitions={'done': 'wait', 'failed': 'armPickupDetectPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:492 y:210
			OperatableStateMachine.add('armPlaceDetectPose',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armPlaceDetectPose", axis_value=["none",0], pos_targets=[], reference_frame="base_arm", end_effector_link="tool0", v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=False, cart_step_list=[3,11], retry_num=3, itp_norm=0.15, if_debug=False, cart_limit={}),
										transitions={'done': 'wait5', 'failed': 'armPlaceDetectPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:329 y:153
			OperatableStateMachine.add('attach',
										SceneManager(action="attach", object_size=[2.278,1.134,0.035], frame_id="tool0", box_name="pvm", box_position=[0,0,0]),
										transitions={'done': 'armPlaceDetectPose'},
										autonomy={'done': Autonomy.Off})

			# x:266 y:31
			OperatableStateMachine.add('detach',
										SceneManager(action="detach", object_size=[2.278,1.134,0.035], frame_id="tool0", box_name="pvm", box_position=[0,0,0]),
										transitions={'done': 'armPickupDetectPose'},
										autonomy={'done': Autonomy.Off})

			# x:462 y:50
			OperatableStateMachine.add('install_pose',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="installpose", axis_value=["none",0], pos_targets=[], reference_frame="base_arm", end_effector_link="tool0", v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=False, cart_step_list=[3,11], retry_num=3, itp_norm=0.15, if_debug=False, cart_limit={}),
										transitions={'done': 'detach', 'failed': 'install_pose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:191 y:216
			OperatableStateMachine.add('test0531',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="test0531", axis_value=["none",0], pos_targets=[], reference_frame="base_arm", end_effector_link="tool0", v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=False, cart_step_list=[3,11], retry_num=3, itp_norm=0.15, if_debug=False, cart_limit={}),
										transitions={'done': 'attach', 'failed': 'test0531'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:17 y:223
			OperatableStateMachine.add('wait',
										WaitState(wait_time=1),
										transitions={'done': 'test0531'},
										autonomy={'done': Autonomy.Off})

			# x:730 y:153
			OperatableStateMachine.add('wait5',
										WaitState(wait_time=1),
										transitions={'done': 'install_pose'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
