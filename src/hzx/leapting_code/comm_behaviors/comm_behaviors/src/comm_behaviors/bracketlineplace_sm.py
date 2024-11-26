#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_behaviors.stoppvmdetect_sm import StopPVMDetectSM
from comm_states.bracket_goal import BracketGoal
from comm_states.manipulation_share import ManipulationShare
from comm_states.publisherheader import PublishHeader
from comm_states.site_manipulation import SiteManipulation
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed May 15 2024
@author: zcx
'''
class BracketLinePlaceSM(Behavior):
	'''
	bkt line place
	'''


	def __init__(self):
		super(BracketLinePlaceSM, self).__init__()
		self.name = 'BracketLinePlace'

		# parameters of this behavior
		self.add_parameter('pvm_width', 1134)
		self.add_parameter('install_gap', 10)
		self.add_parameter('place_solar_z', 0.05)

		# references to used behaviors
		self.add_behavior(StopPVMDetectSM, 'StopPVMDetect')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:273 y:615, x:557 y:576
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['solar_msg'], output_keys=['mani_goal'])
		_state_machine.userdata.mani_goal = {'dx':None}
		_state_machine.userdata.solar_msg = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:118 y:332, x:690 y:143, x:555 y:275, x:330 y:458
		_sm_container_0 = ConcurrencyContainer(outcomes=['finished', 'timeout'], input_keys=['pose_msg'], output_keys=['mani_goal'], conditions=[
										('finished', [('bKtGoal', 'done')]),
										('timeout', [('wait30s', 'done')])
										])

		with _sm_container_0:
			# x:101 y:172
			OperatableStateMachine.add('bKtGoal',
										BracketGoal(pos=[0, (self.pvm_width+ self.install_gap)*0.001, self.place_solar_z], quat=[0, 0, 0, 1], stable_time=3, dx_max=0.05, if_bkt=True, limit_dict={'z_min': -0.3, 'z_max': 0, 'y_min': 0.4, 'y_max': 1.0}),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'pose_msg': 'pose_msg', 'mani_goal': 'mani_goal'})

			# x:243 y:181
			OperatableStateMachine.add('wait30s',
										WaitState(wait_time=30),
										transitions={'done': 'timeout'},
										autonomy={'done': Autonomy.Off})



		with _state_machine:
			# x:77 y:58
			OperatableStateMachine.add('armInit',
										ManipulationShare(reference_frame='base_arm', end_effector_link='tool0'),
										transitions={'done': 'armPlaceBkt1'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:306 y:49
			OperatableStateMachine.add('StopPVMDetect',
										self.use_behavior(StopPVMDetectSM, 'StopPVMDetect'),
										transitions={'finished': 'armPlaceBkt1'},
										autonomy={'finished': Autonomy.Inherit})

			# x:316 y:158
			OperatableStateMachine.add('armPlaceBkt1',
										SiteManipulation(pos=[0.10, (self.pvm_width+ self.install_gap)*0.001+0.35, 0.2], quat=[ 0, 0, 0, 1 ], target_frame='solar_link', target_name='none', axis_value=['none', 0], pos_targets=[], trajectory_name="none", reference_frame='base_arm', end_effector_link='tool0', wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=True, cart_step_list=[1, 15], step_factor=0.1, itp_norm=0, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id='none', plan_time=2),
										transitions={'done': 'startBkt', 'failed': 'armPlaceBkt2'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:547 y:160
			OperatableStateMachine.add('armPlaceBkt2',
										SiteManipulation(pos=[0.1, (self.pvm_width+ self.install_gap)*0.001+0.3, 0.2], quat=[ 0, 0, 0, 1 ], target_frame='solar_link', target_name='none', axis_value=['none', 0], pos_targets=[], trajectory_name="none", reference_frame='base_arm', end_effector_link='tool0', wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=True, cart_step_list=[1, 15], step_factor=0.1, itp_norm=0, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id='none', plan_time=2),
										transitions={'done': 'startBkt', 'failed': 'armPlaceBkt3'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:546 y:252
			OperatableStateMachine.add('armPlaceBkt3',
										SiteManipulation(pos=[0.1, (self.pvm_width+ self.install_gap)*0.001+0.25, 0.2], quat=[ 0, 0, 0, 1 ], target_frame='solar_link', target_name='none', axis_value=['none', 0], pos_targets=[], trajectory_name="none", reference_frame='base_arm', end_effector_link='tool0', wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=True, cart_step_list=[1, 15], step_factor=0.1, itp_norm=0, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id='none', plan_time=2),
										transitions={'done': 'startBkt', 'failed': 'stopBkt'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:254 y:251
			OperatableStateMachine.add('startBkt',
										PublishHeader(seq=1, frame_id="enable_bracket"),
										transitions={'done': 'Container'},
										autonomy={'done': Autonomy.Off})

			# x:552 y:369
			OperatableStateMachine.add('stopBkt',
										PublishHeader(seq=0, frame_id="enable_bracket"),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:244 y:512
			OperatableStateMachine.add('stopBkt1',
										PublishHeader(seq=0, frame_id="enable_bracket"),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:249 y:371
			OperatableStateMachine.add('Container',
										_sm_container_0,
										transitions={'finished': 'stopBkt1', 'timeout': 'stopBkt'},
										autonomy={'finished': Autonomy.Inherit, 'timeout': Autonomy.Inherit},
										remapping={'pose_msg': 'solar_msg', 'mani_goal': 'mani_goal'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
