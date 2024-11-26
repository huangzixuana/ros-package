#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_behaviors.placepvmdetect_sm import PlacePVMDetectSM
from comm_behaviors.startpvmdetect_sm import StartPVMDetectSM
from comm_behaviors.stoppvmdetect_sm import StopPVMDetectSM
from comm_states.listen_solar import ListenSolar
from comm_states.manipulation_share import ManipulationShare
from comm_states.publisherheader import PublishHeader
from comm_states.realtime_goal import RealtimeGoal
from comm_states.site_manipulation import SiteManipulation
from comm_states.trig_decision import TrigDecision
from flexbe_states.decision_state import DecisionState
from flexbe_states.log_state import LogState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jun 05 2023
@author: why
'''
class CommPlacePVMSM(Behavior):
	'''
	PlacePVM
	'''


	def __init__(self):
		super(CommPlacePVMSM, self).__init__()
		self.name = 'CommPlacePVM'

		# parameters of this behavior
		self.add_parameter('mode', 'SimPlacePVM')
		self.add_parameter('pvm_width', 1134)
		self.add_parameter('install_gap', -40)
		self.add_parameter('place_solar_x', -0.03)
		self.add_parameter('place_solar_z', 0.13)
		self.add_parameter('place_ideal_x', 0.6)
		self.add_parameter('place_ideal_y', 2.4)
		self.add_parameter('place_ideal_z', 0.0)
		self.add_parameter('place_ideal_er', -0.05)
		self.add_parameter('place_ideal_ep', -0.04)
		self.add_parameter('place_ideal_ey', 1.63)
		self.add_parameter('place_tol_x', 1.0)
		self.add_parameter('place_tol_y', -1.0)
		self.add_parameter('place_tol_z', -1)
		self.add_parameter('place_tol_er', 0.5)
		self.add_parameter('place_tol_ep', 0.5)
		self.add_parameter('place_tol_ey', 0.5)
		self.add_parameter('detect_option', 'line')
		self.add_parameter('line_gap', 42)
		self.add_parameter('line_z', -0.11)

		# references to used behaviors
		self.add_behavior(PlacePVMDetectSM, 'PlacePVMDetect')
		self.add_behavior(StartPVMDetectSM, 'StartPVMDetect')
		self.add_behavior(StopPVMDetectSM, 'StopPVMDetect')
		self.add_behavior(StopPVMDetectSM, 'StopPVMDetect2')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1246 y:507
		_state_machine = OperatableStateMachine(outcomes=['finished'], output_keys=['nav_goal'])
		_state_machine.userdata.nav_goal = {}
		_state_machine.userdata.mode = self.mode
		_state_machine.userdata.empty_goal = {}
		_state_machine.userdata.detect_option = self.detect_option

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:68 y:59
			OperatableStateMachine.add('armInit',
										ManipulationShare(reference_frame='base_arm', end_effector_link='tool0'),
										transitions={'done': 'armPlaceDetectPose'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:224 y:166
			OperatableStateMachine.add('StartPVMDetect',
										self.use_behavior(StartPVMDetectSM, 'StartPVMDetect'),
										transitions={'finished': 'detectSucceed'},
										autonomy={'finished': Autonomy.Inherit})

			# x:594 y:166
			OperatableStateMachine.add('StopPVMDetect',
										self.use_behavior(StopPVMDetectSM, 'StopPVMDetect'),
										transitions={'finished': 'wait5s'},
										autonomy={'finished': Autonomy.Inherit})

			# x:966 y:490
			OperatableStateMachine.add('StopPVMDetect2',
										self.use_behavior(StopPVMDetectSM, 'StopPVMDetect2'),
										transitions={'finished': 'finished'},
										autonomy={'finished': Autonomy.Inherit})

			# x:222 y:59
			OperatableStateMachine.add('armPlaceDetectPose',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armPlaceDetectPose", axis_value=["none",0], pos_targets=[], trajectory_name='none', reference_frame="base_arm", end_effector_link="tool0", wait_time=2, v_factor=1, a_factor=1, t_factor=1.0, stay_level=False, cart_step_list=[3,11], step_factor=0.1, itp_norm=0.15, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id='none', plan_time=2),
										transitions={'done': 'StartPVMDetect', 'failed': 'armPlaceDetectPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:790 y:452
			OperatableStateMachine.add('armPlacePVM',
										SiteManipulation(pos=[self.place_solar_x, (self.pvm_width+ self.install_gap)*0.001, self.place_solar_z], quat=[0,0,0,1], target_frame="solar_link", target_name="none", axis_value=["none",0], pos_targets=[], trajectory_name='none', reference_frame="base_arm", end_effector_link="tool0", wait_time=0.5, v_factor=0.1, a_factor=0.1, t_factor=1.0, stay_level=True, cart_step_list=[3,11], step_factor=0.1, itp_norm=0.1, retry_num=3, cart_limit={}, if_execute=True, if_debug=True, planner_id='none', plan_time=2),
										transitions={'done': 'StopPVMDetect2', 'failed': 'armPlacePVM'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:247 y:374
			OperatableStateMachine.add('checkPlaceSolar',
										ListenSolar(timeout=60, fresh_time=3.0, ideal=[self.place_ideal_x, self.place_ideal_y, self.place_ideal_z, self.place_ideal_er, self.place_ideal_ep, self.place_ideal_ey], tolerance=[self.place_tol_x, self.place_tol_y, self.place_tol_z, self.place_tol_er, self.place_tol_ep, self.place_ideal_ey], pose_topic='filter_solar_pose'),
										transitions={'done': 'modeDecision', 'timeout': 'detectFailed'},
										autonomy={'done': Autonomy.Off, 'timeout': Autonomy.Off},
										remapping={'pose_msg': 'pose_msg'})

			# x:611 y:377
			OperatableStateMachine.add('dec',
										TrigDecision(),
										transitions={'next': 'StopPVMDetect', 'withdraw': 'dec'},
										autonomy={'next': Autonomy.Off, 'withdraw': Autonomy.Off})

			# x:590 y:525
			OperatableStateMachine.add('detectDec',
										DecisionState(outcomes=['line', 'bkt', 'pvm', 'invalid'], conditions=lambda x: x if (x in ['line', 'bkt', 'pvm']) else 'invalid'),
										transitions={'line': 'PlacePVMDetect', 'bkt': 'armPlacePVM', 'pvm': 'armPlacePVM', 'invalid': 'detectDec'},
										autonomy={'line': Autonomy.Off, 'bkt': Autonomy.Off, 'pvm': Autonomy.Off, 'invalid': Autonomy.Off},
										remapping={'input_value': 'detect_option'})

			# x:441 y:378
			OperatableStateMachine.add('detectFailed',
										PublishHeader(seq=0, frame_id='install_detect_put'),
										transitions={'done': 'dec'},
										autonomy={'done': Autonomy.Off})

			# x:232 y:260
			OperatableStateMachine.add('detectSucceed',
										PublishHeader(seq=1, frame_id="install_detect_put"),
										transitions={'done': 'waitTargetStable2s'},
										autonomy={'done': Autonomy.Off})

			# x:437 y:641
			OperatableStateMachine.add('log2',
										LogState(text='place', severity=Logger.REPORT_HINT),
										transitions={'done': 'stopSOlar'},
										autonomy={'done': Autonomy.Off})

			# x:251 y:491
			OperatableStateMachine.add('modeDecision',
										DecisionState(outcomes=['PlacePVM' ,'SimPlacePVM','invalid'], conditions=lambda x: x if (x in ['PlacePVM' ,'SimPlacePVM']) else 'invalid'),
										transitions={'PlacePVM': 'realtimeGoal*', 'SimPlacePVM': 'log2', 'invalid': 'modeDecision'},
										autonomy={'PlacePVM': Autonomy.Off, 'SimPlacePVM': Autonomy.Off, 'invalid': Autonomy.Off},
										remapping={'input_value': 'mode'})

			# x:251 y:639
			OperatableStateMachine.add('realtimeGoal*',
										RealtimeGoal(position=[1.0,0.75,0], orientation=[0, 0, 0.7071067, 0.7071067], frame_id='solar_link', source_frame='base_link', if_back=False, gap_pvm_pvm=-0.2, gap_pvm_chassis=0.25, chassis_width=2.2, k_y=0, k_yaw=0),
										transitions={'done': 'log2'},
										autonomy={'done': Autonomy.Off},
										remapping={'goal': 'nav_goal'})

			# x:586 y:642
			OperatableStateMachine.add('stopSOlar',
										PublishHeader(seq=0, frame_id="solar_detect"),
										transitions={'done': 'detectDec'},
										autonomy={'done': Autonomy.Off})

			# x:428 y:167
			OperatableStateMachine.add('wait5s',
										WaitState(wait_time=5),
										transitions={'done': 'StartPVMDetect'},
										autonomy={'done': Autonomy.Off})

			# x:43 y:373
			OperatableStateMachine.add('waitTargetStable2s',
										WaitState(wait_time=2),
										transitions={'done': 'checkPlaceSolar'},
										autonomy={'done': Autonomy.Off})

			# x:775 y:573
			OperatableStateMachine.add('PlacePVMDetect',
										self.use_behavior(PlacePVMDetectSM, 'PlacePVMDetect',
											parameters={'place_solar_x': self.place_solar_x, 'pvm_width': self.pvm_width, 'install_gap': self.install_gap, 'place_solar_z': self.place_solar_z, 'en_bracket': False, 'en_plane': True, 'yline_gap': self.line_gap, 'line_z': self.line_z}),
										transitions={'finished': 'StopPVMDetect2'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'solar_msg': 'pose_msg'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
