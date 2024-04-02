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
		self.add_parameter('install_gap', 10)
		self.add_parameter('place_solar_x', 0)
		self.add_parameter('place_solar_z', 0.03)
		self.add_parameter('place_ideal_x', 0.6)
		self.add_parameter('place_ideal_y', 2.4)
		self.add_parameter('place_ideal_z', 0)
		self.add_parameter('place_ideal_er', -0.05)
		self.add_parameter('place_ideal_ep', -0.04)
		self.add_parameter('place_ideal_ey', 1.63)
		self.add_parameter('place_tol_x', 0.5)
		self.add_parameter('place_tol_y', 0.3)
		self.add_parameter('place_tol_z', -1)
		self.add_parameter('place_tol_er', 0.5)
		self.add_parameter('place_tol_ep', 0.5)
		self.add_parameter('place_tol_ey', 0.5)

		# references to used behaviors
		self.add_behavior(StartPVMDetectSM, 'StartPVMDetect')
		self.add_behavior(StopPVMDetectSM, 'StopPVMDetect')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:890 y:412
		_state_machine = OperatableStateMachine(outcomes=['finished'], output_keys=['nav_goal'])
		_state_machine.userdata.nav_goal = {}
		_state_machine.userdata.mode = self.mode
		_state_machine.userdata.empty_goal = {}

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:76 y:54
			OperatableStateMachine.add('armInit',
										ManipulationShare(reference_frame='base_arm', end_effector_link='tool0'),
										transitions={'done': 'log'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:439 y:145
			OperatableStateMachine.add('StartPVMDetect',
										self.use_behavior(StartPVMDetectSM, 'StartPVMDetect'),
										transitions={'finished': 'checkPlaceSOlar'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'nav_goal': 'nav_goal'})

			# x:839 y:145
			OperatableStateMachine.add('StopPVMDetect',
										self.use_behavior(StopPVMDetectSM, 'StopPVMDetect'),
										transitions={'finished': 'wait5s'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'nav_goal': 'nav_goal'})

			# x:440 y:51
			OperatableStateMachine.add('armPlaceDetectPose',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armPlaceDetectPose", axis_value=["none",0], pos_targets=[], reference_frame="base_arm", end_effector_link="tool0", v_factor=1, a_factor=1, if_execute=True, wait_time=2, stay_level=False, cart_step_list=[3,11], retry_num=3, itp_norm=0.15, if_debug=False),
										transitions={'done': 'StartPVMDetect', 'failed': 'armPlaceDetectPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:403 y:484
			OperatableStateMachine.add('armPlacePVM',
										SiteManipulation(pos=[self.place_solar_x, (self.pvm_width+ self.install_gap)*0.001, self.place_solar_z], quat=[0, 0, 0, 1], target_frame="solar_link", target_name="none", axis_value=["none",0], pos_targets=[], reference_frame="base_arm", end_effector_link="tool0", v_factor=0.1, a_factor=0.1, if_execute=True, wait_time=0.5, stay_level=True, cart_step_list=[3,11], retry_num=3, itp_norm=0.1, if_debug=True),
										transitions={'done': 'stopPlaceSegment', 'failed': 'armPlacePVM'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:204 y:151
			OperatableStateMachine.add('checkPlaceSOlar',
										ListenSolar(timeout=60, fresh_time=3.0, ideal=[self.place_ideal_x, self.place_ideal_y, self.place_ideal_z, self.place_ideal_er, self.place_ideal_ep, self.place_ideal_ey], tolerance=[self.place_tol_x, self.place_tol_y, self.place_tol_z, self.place_tol_er, self.place_tol_ep, self.place_ideal_ey]),
										transitions={'done': 'detectSucceed', 'timeout': 'detectFailed'},
										autonomy={'done': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:866 y:265
			OperatableStateMachine.add('dec',
										TrigDecision(),
										transitions={'next': 'StopPVMDetect', 'withdraw': 'dec'},
										autonomy={'next': Autonomy.Off, 'withdraw': Autonomy.Off})

			# x:482 y:259
			OperatableStateMachine.add('detectFailed',
										PublishHeader(seq=0, frame_id='install_detect_put'),
										transitions={'done': 'dec'},
										autonomy={'done': Autonomy.Off})

			# x:208 y:273
			OperatableStateMachine.add('detectSucceed',
										PublishHeader(seq=1, frame_id="install_detect_put"),
										transitions={'done': 'waitTargetStable5s'},
										autonomy={'done': Autonomy.Off})

			# x:268 y:50
			OperatableStateMachine.add('log',
										LogState(text="gap: %s"%str(self.install_gap), severity=Logger.REPORT_HINT),
										transitions={'done': 'armPlaceDetectPose'},
										autonomy={'done': Autonomy.Off})

			# x:71 y:496
			OperatableStateMachine.add('modeDecision',
										DecisionState(outcomes=['PlacePVM' ,'SimPlacePVM','invalid'], conditions=lambda x: x if (x in ['PlacePVM' ,'SimPlacePVM']) else 'invalid'),
										transitions={'PlacePVM': 'realtimeGoal*', 'SimPlacePVM': 'LOG', 'invalid': 'modeDecision'},
										autonomy={'PlacePVM': Autonomy.Off, 'SimPlacePVM': Autonomy.Off, 'invalid': Autonomy.Off},
										remapping={'input_value': 'mode'})

			# x:78 y:610
			OperatableStateMachine.add('realtimeGoal*',
										RealtimeGoal(position=[1.0,0.75,0], orientation=[0, 0, 0.7071, 0.7071], frame_id='solar_link', source_frame='base_link', if_back=False, gap_pvm_pvm=0.1, gap_pvm_chassis=0, chassis_width=2.2),
										transitions={'done': 'LOG'},
										autonomy={'done': Autonomy.Off},
										remapping={'goal': 'nav_goal'})

			# x:696 y:393
			OperatableStateMachine.add('stopPlaceDetect',
										PublishHeader(seq=0, frame_id="solar_detect"),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:692 y:482
			OperatableStateMachine.add('stopPlaceSegment',
										PublishHeader(seq=0, frame_id="enable_yolov8"),
										transitions={'done': 'stopPlaceDetect'},
										autonomy={'done': Autonomy.Off})

			# x:678 y:146
			OperatableStateMachine.add('wait5s',
										WaitState(wait_time=5),
										transitions={'done': 'StartPVMDetect'},
										autonomy={'done': Autonomy.Off})

			# x:60 y:272
			OperatableStateMachine.add('waitTargetStable5s',
										WaitState(wait_time=2),
										transitions={'done': 'modeDecision'},
										autonomy={'done': Autonomy.Off})

			# x:253 y:608
			OperatableStateMachine.add('LOG',
										LogState(text='place', severity=Logger.REPORT_HINT),
										transitions={'done': 'armPlacePVM'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
