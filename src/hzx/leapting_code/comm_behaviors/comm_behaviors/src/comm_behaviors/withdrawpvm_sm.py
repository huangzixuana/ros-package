#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_behaviors.cupoff_sm import CupOffSM
from comm_states.manipulation_share import ManipulationShare
from comm_states.publisherheader import PublishHeader
from comm_states.scene_manager import SceneManager
from comm_states.site_manipulation import SiteManipulation
from comm_states.site_navigation import SiteNavigation
from flexbe_states.decision_state import DecisionState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jun 02 2023
@author: why
'''
class WithdrawPVMSM(Behavior):
	'''
	Withdraw PVM to the trailer
	'''


	def __init__(self):
		super(WithdrawPVMSM, self).__init__()
		self.name = 'WithdrawPVM'

		# parameters of this behavior
		self.add_parameter('install_mode', True)

		# references to used behaviors
		self.add_behavior(CupOffSM, 'CupOff')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1156 y:217
		_state_machine = OperatableStateMachine(outcomes=['finished'], input_keys=['nav_goal'])
		_state_machine.userdata.install_mode = self.install_mode
		_state_machine.userdata.nav_goal = {}
		_state_machine.userdata.empty_goal = {}

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('armInit',
										ManipulationShare(reference_frame='base_arm', end_effector_link='tool0'),
										transitions={'done': 'att11'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:56 y:323
			OperatableStateMachine.add('armPickDetect',
										SiteManipulation(pos=[0, 0, 0], quat=[0, 0, 0, 1], target_frame='none', target_name='armPickDetect', axis_value=['none', 0], pos_targets=[], trajectory_name='none', reference_frame='base_arm', end_effector_link='tool0', wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=False, cart_step_list=[3, 11], step_factor=0.1, itp_norm=0.15, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id='RRTConnect', plan_time=2),
										transitions={'done': 'CupOff', 'failed': 'armPickDetect'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:1001 y:297
			OperatableStateMachine.add('armTOPPOSE',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armTopPose", axis_value=["none",0], pos_targets=[], trajectory_name='none', reference_frame="base_arm", end_effector_link="tool0", wait_time=1, v_factor=1, a_factor=1, t_factor=1.0, stay_level=False, cart_step_list=[3, 11], step_factor=0.1, itp_norm=0.15, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id='RRTConnect', plan_time=2),
										transitions={'done': 'finished', 'failed': 'armTOPPOSE'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:27 y:186
			OperatableStateMachine.add('armTopPose',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armTopPose", axis_value=["none",0], pos_targets=[], trajectory_name="none", reference_frame="base_arm", end_effector_link="tool0", wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=False, cart_step_list=[3,11], step_factor=0.1, itp_norm=0.15, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id="LazyPRMstar", plan_time=2),
										transitions={'done': 'armPickDetect', 'failed': 'armTopPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:991 y:101
			OperatableStateMachine.add('armTopPose2',
										SiteManipulation(pos=[1.9780,-0.0493,1.6400], quat=[0.0009615,-0.00208041,0.71054785,0.70364515], target_frame="none", target_name="none", axis_value=["none",0], pos_targets=[], trajectory_name="none", reference_frame="base_arm", end_effector_link="tool0", wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=True, cart_step_list=[1,11], step_factor=0.1, itp_norm=0, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id="none", plan_time=2),
										transitions={'done': 'finished', 'failed': 'armTopPose2'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:31 y:111
			OperatableStateMachine.add('att11',
										SceneManager(action="attach", object_size=[2.278,1.134,0.035], frame_id="tool0", box_name="pvm", box_position=[0,0,0]),
										transitions={'done': 'armTopPose'},
										autonomy={'done': Autonomy.Off})

			# x:330 y:160
			OperatableStateMachine.add('att22',
										SceneManager(action="attach", object_size=[1.8,1.1,0.035], frame_id="tool0", box_name="pvm1", box_position=[0,0,0]),
										transitions={'done': 'stopWithdrawSegment'},
										autonomy={'done': Autonomy.Off})

			# x:592 y:229
			OperatableStateMachine.add('baseBackward',
										SiteNavigation(site_name="", position=[0,0,0], orientation=[0,0,0,1], frame_id="map", base_link2map=True),
										transitions={'arrived': 'waitBack1s', 'canceled': 'baseBackward', 'failed': 'baseBackward'},
										autonomy={'arrived': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'nav_goal': 'nav_goal'})

			# x:722 y:146
			OperatableStateMachine.add('decision',
										DecisionState(outcomes=['install', 'uninstall'], conditions=lambda x: 'install' if x else 'uninstall'),
										transitions={'install': 'finished', 'uninstall': 'wait5s'},
										autonomy={'install': Autonomy.Off, 'uninstall': Autonomy.Off},
										remapping={'input_value': 'install_mode'})

			# x:502 y:358
			OperatableStateMachine.add('nav-back',
										SiteNavigation(site_name="", position=[-1,0,0], orientation=[0,0,0,1], frame_id="base_link", base_link2map=False),
										transitions={'arrived': 'waitBack1s', 'canceled': 'nav-back', 'failed': 'nav-back'},
										autonomy={'arrived': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'nav_goal': 'empty_goal'})

			# x:497 y:42
			OperatableStateMachine.add('stopWithdrawDetect',
										PublishHeader(seq=0, frame_id="solar_detect"),
										transitions={'done': 'decision'},
										autonomy={'done': Autonomy.Off})

			# x:319 y:42
			OperatableStateMachine.add('stopWithdrawSegment',
										PublishHeader(seq=0, frame_id="enable_yolov8"),
										transitions={'done': 'stopWithdrawDetect'},
										autonomy={'done': Autonomy.Off})

			# x:501 y:145
			OperatableStateMachine.add('wait5s',
										WaitState(wait_time=5),
										transitions={'done': 'nav-back'},
										autonomy={'done': Autonomy.Off})

			# x:808 y:269
			OperatableStateMachine.add('waitBack1s',
										WaitState(wait_time=1),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:322 y:297
			OperatableStateMachine.add('CupOff',
										self.use_behavior(CupOffSM, 'CupOff'),
										transitions={'finished': 'att22'},
										autonomy={'finished': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
