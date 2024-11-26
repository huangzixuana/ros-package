#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_behaviors.startpvmdetect_sm import StartPVMDetectSM
from comm_behaviors.stoppvmdetect_sm import StopPVMDetectSM
from comm_states.bracket_goal import BracketGoal
from comm_states.listen_solar import ListenSolar
from comm_states.manipulation_share import ManipulationShare
from comm_states.publisherheader import PublishHeader
from comm_states.site_manipulation import SiteManipulation
from comm_states.trig_decision import TrigDecision
from flexbe_states.decision_state import DecisionState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jun 15 2023
@author: zl
'''
class TrailerPVMDetectSM(Behavior):
	'''
	Detect PVM in trailer
	'''


	def __init__(self):
		super(TrailerPVMDetectSM, self).__init__()
		self.name = 'TrailerPVMDetect'

		# parameters of this behavior
		self.add_parameter('pick_solar_x', 0.000001)
		self.add_parameter('pick_solar_y', 0.08)
		self.add_parameter('pick_solar_z', 0.05)
		self.add_parameter('pick_ideal_x', 2.086)
		self.add_parameter('pick_ideal_y', -0.127)
		self.add_parameter('pick_ideal_z', 0.0)
		self.add_parameter('pick_ideal_er', 0.006)
		self.add_parameter('pick_ideal_ep', -0.011)
		self.add_parameter('pick_ideal_ey', 1.559)
		self.add_parameter('pick_tol_x', -1.0)
		self.add_parameter('pick_tol_y', -1.0)
		self.add_parameter('pick_tol_z', -1.0)
		self.add_parameter('pick_tol_er', 0.2)
		self.add_parameter('pick_tol_ep', 0.2)
		self.add_parameter('pick_tol_ey', 0.2)
		self.add_parameter('detect_option', 'line')

		# references to used behaviors
		self.add_behavior(StartPVMDetectSM, 'StartPVMDetect')
		self.add_behavior(StopPVMDetectSM, 'StopPVMDetect')
		self.add_behavior(StopPVMDetectSM, 'StopPVMDetect2')
		self.add_behavior(StopPVMDetectSM, 'StopPVMDetect_2')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:927 y:246
		_state_machine = OperatableStateMachine(outcomes=['finished'], input_keys=['nav_goal', 'pose_msg_in'], output_keys=['pose_msg_pick'])
		_state_machine.userdata.nav_goal = {}
		_state_machine.userdata.detect_option = self.detect_option
		_state_machine.userdata.pose_msg_in = None
		_state_machine.userdata.pose_msg_pick = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:58 y:74
			OperatableStateMachine.add('armInit',
										ManipulationShare(reference_frame='base_arm', end_effector_link='tool0'),
										transitions={'done': 'detectDec'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:30 y:317
			OperatableStateMachine.add('StopPVMDetect',
										self.use_behavior(StopPVMDetectSM, 'StopPVMDetect'),
										transitions={'finished': 'StartPVMDetect'},
										autonomy={'finished': Autonomy.Inherit})

			# x:875 y:314
			OperatableStateMachine.add('StopPVMDetect2',
										self.use_behavior(StopPVMDetectSM, 'StopPVMDetect2'),
										transitions={'finished': 'finished'},
										autonomy={'finished': Autonomy.Inherit})

			# x:608 y:573
			OperatableStateMachine.add('StopPVMDetect_2',
										self.use_behavior(StopPVMDetectSM, 'StopPVMDetect_2'),
										transitions={'finished': 'pos2Mani'},
										autonomy={'finished': Autonomy.Inherit})

			# x:252 y:153
			OperatableStateMachine.add('armPickupDetectPose',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armPickupDetectPose", axis_value=["none",0], pos_targets=[], trajectory_name="none", reference_frame="base_arm", end_effector_link="tool0", wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=False, cart_step_list=[3,11], step_factor=0.1, itp_norm=0.15, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id='none', plan_time=2),
										transitions={'done': 'wait2sCamDelay', 'failed': 'armPickupDetectPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:880 y:435
			OperatableStateMachine.add('armPickupThrough',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="none", axis_value=["none",0], pos_targets=['armTopPose', 'temp_itp', 'temp_solar'], trajectory_name="none", reference_frame="base_arm", end_effector_link="tool0", wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=True, cart_step_list=[3,18], step_factor=0.1, itp_norm=0.0, retry_num=30, cart_limit={}, if_execute=True, if_debug=False, planner_id="none", plan_time=2),
										transitions={'done': 'StopPVMDetect2', 'failed': 'wait1Ks'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:250 y:577
			OperatableStateMachine.add('checkPickupSolar',
										ListenSolar(timeout=60, fresh_time=3.0, ideal=[self.pick_ideal_x, self.pick_ideal_y, self.pick_ideal_z, self.pick_ideal_er, self.pick_ideal_ep, self.pick_ideal_ey], tolerance=[self.pick_tol_x, self.pick_tol_y, self.pick_tol_z, self.pick_tol_er, self.pick_tol_ep, self.pick_tol_ey], pose_topic='filter_solar_pose'),
										transitions={'done': 'detectSucceed', 'timeout': 'detectFailed'},
										autonomy={'done': Autonomy.Off, 'timeout': Autonomy.Off},
										remapping={'pose_msg': 'pose_msg_pick'})

			# x:42 y:407
			OperatableStateMachine.add('dec1',
										TrigDecision(),
										transitions={'next': 'StopPVMDetect', 'withdraw': 'dec1'},
										autonomy={'next': Autonomy.Off, 'withdraw': Autonomy.Off})

			# x:267 y:75
			OperatableStateMachine.add('detectDec',
										DecisionState(outcomes=['line','sec', 'pvm', 'invalid'], conditions=lambda x: x if (x in ['line','sec', 'pvm']) else 'invalid'),
										transitions={'line': 'armPickupDetectPose', 'sec': 'pose2Mani2nd', 'pvm': 'armPickupDetectPose', 'invalid': 'detectDec'},
										autonomy={'line': Autonomy.Off, 'sec': Autonomy.Off, 'pvm': Autonomy.Off, 'invalid': Autonomy.Off},
										remapping={'input_value': 'detect_option'})

			# x:38 y:580
			OperatableStateMachine.add('detectFailed',
										PublishHeader(seq=0, frame_id="install_detect_pick"),
										transitions={'done': 'waitPickupTarget3s'},
										autonomy={'done': Autonomy.Off})

			# x:432 y:578
			OperatableStateMachine.add('detectSucceed',
										PublishHeader(seq=1, frame_id="install_detect_pick"),
										transitions={'done': 'StopPVMDetect_2'},
										autonomy={'done': Autonomy.Off})

			# x:631 y:437
			OperatableStateMachine.add('pos2Mani',
										BracketGoal(pos=[self.pick_solar_x, self.pick_solar_y, self.pick_solar_z], quat=[0,0.0130896,0,0.9999143 ], dx_max=0.1, limit_dict={}, pos_targets=['armTopPose'], itp_norm=0.15),
										transitions={'done': 'armPickupThrough'},
										autonomy={'done': Autonomy.Off},
										remapping={'pose_msg': 'pose_msg_pick', 'mani_goal': 'mani_goal_pick'})

			# x:624 y:328
			OperatableStateMachine.add('pose2Mani2nd',
										BracketGoal(pos=[self.pick_solar_x, self.pick_solar_y, self.pick_solar_z-0.03], quat=[0,0.0130896,0,0.9999143], dx_max=0.1, limit_dict={}, pos_targets=['armTopPose'], itp_norm=0.15),
										transitions={'done': 'armPickupThrough'},
										autonomy={'done': Autonomy.Off},
										remapping={'pose_msg': 'pose_msg_in', 'mani_goal': 'mani_goal_pick'})

			# x:893 y:543
			OperatableStateMachine.add('wait1Ks',
										WaitState(wait_time=1000),
										transitions={'done': 'wait1Ks'},
										autonomy={'done': Autonomy.Off})

			# x:255 y:236
			OperatableStateMachine.add('wait2sCamDelay',
										WaitState(wait_time=2),
										transitions={'done': 'StartPVMDetect'},
										autonomy={'done': Autonomy.Off})

			# x:29 y:488
			OperatableStateMachine.add('waitPickupTarget3s',
										WaitState(wait_time=3),
										transitions={'done': 'dec1'},
										autonomy={'done': Autonomy.Off})

			# x:250 y:458
			OperatableStateMachine.add('waitTargetStable2s',
										WaitState(wait_time=2),
										transitions={'done': 'checkPickupSolar'},
										autonomy={'done': Autonomy.Off})

			# x:249 y:315
			OperatableStateMachine.add('StartPVMDetect',
										self.use_behavior(StartPVMDetectSM, 'StartPVMDetect'),
										transitions={'finished': 'waitTargetStable2s'},
										autonomy={'finished': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
