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
from comm_behaviors.fromplace2pick_sm import fromPlace2PickSM
from comm_states.listen_state import ListenState
from comm_states.manipulation_share import ManipulationShare
from comm_states.publisherheader import PublishHeader
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
		self.add_behavior(fromPlace2PickSM, 'fromPlace2Pick')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1181 y:259
		_state_machine = OperatableStateMachine(outcomes=['finished'], input_keys=['nav_goal'])
		_state_machine.userdata.install_mode = self.install_mode
		_state_machine.userdata.nav_goal = {}

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('armInit',
										ManipulationShare(reference_frame='base_arm', end_effector_link='tool0'),
										transitions={'done': 'fromPlace2Pick'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:722 y:146
			OperatableStateMachine.add('Decision',
										DecisionState(outcomes=['install', 'uninstall'], conditions=lambda x: 'install' if x else 'uninstall'),
										transitions={'install': 'armPickupDetectPose1', 'uninstall': 'wait5s'},
										autonomy={'install': Autonomy.Off, 'uninstall': Autonomy.Off},
										remapping={'input_value': 'install_mode'})

			# x:30 y:301
			OperatableStateMachine.add('armPickDetect',
										SiteManipulation(pos=[0, 0, 0], quat=[0, 0, 0, 1], target_frame='none', target_name='armPickDetect', axis_value=['none', 0], pos_targets=[], reference_frame='base_arm', end_effector_link='tool0', v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=False, cart_step_list=[3, 11], retry_num=3, itp_norm=0.15, if_debug=False),
										transitions={'done': 'CupOff', 'failed': 'armPickDetect'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:876 y:249
			OperatableStateMachine.add('armPickupDetectPose1',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armPickupDetectPose", axis_value=["none",0], pos_targets=[], reference_frame="base_arm", end_effector_link="tool0", v_factor=1, a_factor=1, if_execute=True, wait_time=1, stay_level=False, cart_step_list=[3, 11], retry_num=3, itp_norm=0.15, if_debug=False),
										transitions={'done': 'finished', 'failed': 'armPickupDetectPose1'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:25 y:211
			OperatableStateMachine.add('armPickupMidPose',
										SiteManipulation(pos=[0, 0, 0], quat=[0, 0, 0, 1], target_frame='none', target_name='armPickupMidPose', axis_value=['none', 0], pos_targets=[], reference_frame='base_arm', end_effector_link='tool0', v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=False, cart_step_list=[3, 11], retry_num=3, itp_norm=0.15, if_debug=False),
										transitions={'done': 'armPickDetect', 'failed': 'armPickupMidPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:320 y:422
			OperatableStateMachine.add('armWithdrawPVM',
										SiteManipulation(pos=[0.0,0,0.03], quat=[0, 0.02179, 0, 0.9997], target_frame="solar_link", target_name="none", axis_value=["none",0], pos_targets=[], reference_frame="base_arm", end_effector_link="tool0", v_factor=0.2, a_factor=0.2, if_execute=True, wait_time=4, stay_level=True, cart_step_list=[3, 11], retry_num=3, itp_norm=0.3, if_debug=False),
										transitions={'done': 'CupOff', 'failed': 'armWithdrawPVM'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:497 y:255
			OperatableStateMachine.add('baseBackward',
										SiteNavigation(site_name="", position=[0,0,0], orientation=[0,0,0,1], frame_id="map"),
										transitions={'arrived': 'waitBack', 'canceled': 'baseBackward', 'failed': 'baseBackward'},
										autonomy={'arrived': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'nav_goal': 'nav_goal'})

			# x:322 y:638
			OperatableStateMachine.add('checkWithdrawTF',
										ListenState(target_frame='solar_link', source_frame='base_arm', timeout=60, fresh_time=3.0, ideal=[0, 0, 0, 0, 0, 0], tolerance=[-1, -1, -1, -1, -1, -1]),
										transitions={'done': 'waitTargetStable', 'timeout': 'waitWithdrawTarget'},
										autonomy={'done': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:22 y:120
			OperatableStateMachine.add('fromPlace2Pick',
										self.use_behavior(fromPlace2PickSM, 'fromPlace2Pick'),
										transitions={'finished': 'armPickupMidPose'},
										autonomy={'finished': Autonomy.Inherit})

			# x:36 y:638
			OperatableStateMachine.add('startWithdrawDetect',
										PublishHeader(seq=2, frame_id="solar_detect"),
										transitions={'done': 'checkWithdrawTF'},
										autonomy={'done': Autonomy.Off})

			# x:22 y:461
			OperatableStateMachine.add('startWithdrawSegment',
										PublishHeader(seq=2, frame_id="enable_yolov8"),
										transitions={'done': 'startWithdrawDetect'},
										autonomy={'done': Autonomy.Off})

			# x:497 y:42
			OperatableStateMachine.add('stopWithdrawDetect',
										PublishHeader(seq=0, frame_id="solar_detect"),
										transitions={'done': 'Decision'},
										autonomy={'done': Autonomy.Off})

			# x:319 y:42
			OperatableStateMachine.add('stopWithdrawSegment',
										PublishHeader(seq=0, frame_id="enable_yolov8"),
										transitions={'done': 'stopWithdrawDetect'},
										autonomy={'done': Autonomy.Off})

			# x:496 y:146
			OperatableStateMachine.add('wait5s',
										WaitState(wait_time=5),
										transitions={'done': 'baseBackward'},
										autonomy={'done': Autonomy.Off})

			# x:724 y:252
			OperatableStateMachine.add('waitBack',
										WaitState(wait_time=5),
										transitions={'done': 'armPickupDetectPose1'},
										autonomy={'done': Autonomy.Off})

			# x:326 y:537
			OperatableStateMachine.add('waitTargetStable',
										WaitState(wait_time=1),
										transitions={'done': 'armWithdrawPVM'},
										autonomy={'done': Autonomy.Off})

			# x:143 y:527
			OperatableStateMachine.add('waitWithdrawTarget',
										WaitState(wait_time=3),
										transitions={'done': 'startWithdrawSegment'},
										autonomy={'done': Autonomy.Off})

			# x:322 y:295
			OperatableStateMachine.add('CupOff',
										self.use_behavior(CupOffSM, 'CupOff'),
										transitions={'finished': 'stopWithdrawSegment'},
										autonomy={'finished': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
