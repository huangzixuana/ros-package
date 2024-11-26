#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_behaviors.handeyeweb_sm import HandEyeWebSM
from comm_states.calibration_site import CalibrationSite
from comm_states.handeye_action import HandeyeAction as comm_states__HandeyeAction
from comm_states.listen_tag import ListenTag
from comm_states.manipulation_share import ManipulationShare
from comm_states.publish_string import PublishString
from comm_states.roslaunch_node import roslaunch_node
from comm_states.roslaunch_node_shut import roslaunch_node_shut
from comm_states.site_manipulation import SiteManipulation
from comm_states.site_manipulation_cali import SiteManipulationCali
from flexbe_states.calculation_state import CalculationState
from flexbe_states.decision_state import DecisionState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Oct 31 2023
@author: zcx
'''
class HandEyeCalibration_testSM(Behavior):
	'''
	calibrate hand eye
	'''


	def __init__(self):
		super(HandEyeCalibration_testSM, self).__init__()
		self.name = 'HandEyeCalibration_test'

		# parameters of this behavior
		self.add_parameter('if_auto_all', False)
		self.add_parameter('start_site', 1)
		self.add_parameter('end_site', 1)
		self.add_parameter('mirror', False)

		# references to used behaviors
		self.add_behavior(HandEyeWebSM, 'MoveWeb')
		self.add_behavior(HandEyeWebSM, 'MoveWeb2')
		self.add_behavior(HandEyeWebSM, 'TakeWeb')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1675 y:67
		_state_machine = OperatableStateMachine(outcomes=['finished'])
		_state_machine.userdata.sampler = None
		_state_machine.userdata.auto = self.if_auto_all
		_state_machine.userdata.no_site = self.start_site

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:86 y:55
			OperatableStateMachine.add('armInit',
										ManipulationShare(reference_frame="base_arm", end_effector_link="tool0"),
										transitions={'done': 'launchCalibrate'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:319 y:565
			OperatableStateMachine.add('MoveWeb',
										self.use_behavior(HandEyeWebSM, 'MoveWeb',
											parameters={'web_info': "UI_handeye_arm:next?"}),
										transitions={'finished': 'calibrationPose'},
										autonomy={'finished': Autonomy.Inherit})

			# x:1507 y:165
			OperatableStateMachine.add('MoveWeb2',
										self.use_behavior(HandEyeWebSM, 'MoveWeb2',
											parameters={'web_info': "UI_handeye_arm:next?"}),
										transitions={'finished': 'armPickupDetectPose'},
										autonomy={'finished': Autonomy.Inherit})

			# x:674 y:421
			OperatableStateMachine.add('TakeWeb',
										self.use_behavior(HandEyeWebSM, 'TakeWeb',
											parameters={'web_info': "UI_handeye_take:next?"}),
										transitions={'finished': 'takeSample'},
										autonomy={'finished': Autonomy.Inherit})

			# x:1278 y:59
			OperatableStateMachine.add('armPickupDetectPose',
										SiteManipulation(pos=[0, 0, 0], quat=[0, 0, 0, 1], target_frame='none', target_name='armPickupDetectPose', axis_value=['none', 0], pos_targets=[], trajectory_name='none', reference_frame='base_arm', end_effector_link='tool0', wait_time=3, v_factor=1, a_factor=1, t_factor=1.0, stay_level=False, cart_step_list=[3, 8], step_factor=0.1, itp_norm=0, retry_num=3, cart_limit={}, if_execute=True, if_debug=True, planner_id='none', plan_time=2),
										transitions={'done': 'finished', 'failed': 'armPickupDetectPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:1270 y:283
			OperatableStateMachine.add('armPickupDetectPosePlan',
										SiteManipulation(pos=[0, 0, 0], quat=[0, 0, 0, 1], target_frame='none', target_name='armPickupDetectPose', axis_value=['none', 0], pos_targets=[], trajectory_name='none', reference_frame='base_arm', end_effector_link='tool0', wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=False, cart_step_list=[3, 8], step_factor=0.1, itp_norm=0, retry_num=3, cart_limit={}, if_execute=False, if_debug=False, planner_id='none', plan_time=2),
										transitions={'done': 'ifAuto', 'failed': 'armPickupDetectPosePlan'},
										autonomy={'done': Autonomy.High, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:322 y:422
			OperatableStateMachine.add('calibrationPose',
										SiteManipulationCali(calibration_site="", reference_frame="base_arm", end_effector_link="tool0", wait_time=0, v_factor=1, a_factor=1, if_execute=True, if_debug=False),
										transitions={'done': 'wait6s', 'failed': 'calibrationPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'arm_site': 'arm_site', 'no_site': 'no_site'})

			# x:77 y:423
			OperatableStateMachine.add('calibrationPosePlan',
										SiteManipulationCali(calibration_site="", reference_frame="base_arm", end_effector_link="tool0", wait_time=0, v_factor=1, a_factor=1, if_execute=False, if_debug=False),
										transitions={'done': 'moveDec', 'failed': 'calibrationPosePlan'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'arm_site': 'arm_site', 'no_site': 'no_site'})

			# x:1293 y:473
			OperatableStateMachine.add('calibration_10',
										SiteManipulationCali(calibration_site="10", reference_frame="base_arm", end_effector_link="tool0", wait_time=0, v_factor=1, a_factor=1, if_execute=True, if_debug=False),
										transitions={'done': 'calibration_5', 'failed': 'calibration_10'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'arm_site': 'arm_site', 'no_site': 'no_site'})

			# x:1292 y:569
			OperatableStateMachine.add('calibration_15',
										SiteManipulationCali(calibration_site="15", reference_frame="base_arm", end_effector_link="tool0", wait_time=0, v_factor=1, a_factor=1, if_execute=True, if_debug=False),
										transitions={'done': 'calibration_10', 'failed': 'calibration_15'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'arm_site': 'arm_site', 'no_site': 'no_site'})

			# x:1018 y:568
			OperatableStateMachine.add('calibration_16',
										SiteManipulationCali(calibration_site="16", reference_frame="base_arm", end_effector_link="tool0", wait_time=0, v_factor=1, a_factor=1, if_execute=True, if_debug=False),
										transitions={'done': 'calibration_15', 'failed': 'calibration_16'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'arm_site': 'arm_site', 'no_site': 'no_site'})

			# x:1293 y:385
			OperatableStateMachine.add('calibration_5',
										SiteManipulationCali(calibration_site="5", reference_frame="base_arm", end_effector_link="tool0", wait_time=0, v_factor=1, a_factor=1, if_execute=True, if_debug=False),
										transitions={'done': 'armPickupDetectPosePlan', 'failed': 'calibration_5'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'arm_site': 'arm_site', 'no_site': 'no_site'})

			# x:522 y:570
			OperatableStateMachine.add('firstDec',
										DecisionState(outcomes=['first', 'next'], conditions=lambda x: 'first' if x==self.start_site else 'next'),
										transitions={'first': 'listenTag', 'next': 'takeDec'},
										autonomy={'first': Autonomy.Off, 'next': Autonomy.Off},
										remapping={'input_value': 'no_site'})

			# x:862 y:215
			OperatableStateMachine.add('helpClick',
										PublishString(name="easy_handeye", value="take"),
										transitions={'done': 'poseDec'},
										autonomy={'done': Autonomy.Off})

			# x:1308 y:167
			OperatableStateMachine.add('ifAuto',
										DecisionState(outcomes=['True', 'False'], conditions=lambda x: 'True' if x else 'False'),
										transitions={'True': 'armPickupDetectPose', 'False': 'MoveWeb2'},
										autonomy={'True': Autonomy.Off, 'False': Autonomy.Off},
										remapping={'input_value': 'auto'})

			# x:408 y:141
			OperatableStateMachine.add('launchApriltag',
										roslaunch_node(cmd='roslaunch', pkg='bringup', launch_Node='apriltag.launch'),
										transitions={'done': 'wait18s'},
										autonomy={'done': Autonomy.Off},
										remapping={'output_value': 'output_value', 'shutdown_class': 'shutdown_class2'})

			# x:256 y:55
			OperatableStateMachine.add('launchCalibrate',
										roslaunch_node(cmd='roslaunch', pkg='bringup', launch_Node='calibrate.launch'),
										transitions={'done': 'wait10s'},
										autonomy={'done': Autonomy.Off},
										remapping={'output_value': 'output_value', 'shutdown_class': 'shutdown_class1'})

			# x:524 y:703
			OperatableStateMachine.add('listenTag',
										ListenTag(),
										transitions={'done': 'takeDec'},
										autonomy={'done': Autonomy.Off})

			# x:85 y:571
			OperatableStateMachine.add('moveDec',
										DecisionState(outcomes=['True', 'False'], conditions=lambda x: 'True' if x else 'False'),
										transitions={'True': 'calibrationPose', 'False': 'MoveWeb'},
										autonomy={'True': Autonomy.Off, 'False': Autonomy.Off},
										remapping={'input_value': 'auto'})

			# x:400 y:274
			OperatableStateMachine.add('poseAdd',
										CalculationState(calculation=lambda x: x+1),
										transitions={'done': 'calibrationPosePlan'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'no_site', 'output_value': 'no_site'})

			# x:861 y:58
			OperatableStateMachine.add('poseDec',
										DecisionState(outcomes=['next','finish'], conditions=lambda x: 'next' if x<self.end_site else 'finish'),
										transitions={'next': 'poseAdd', 'finish': 'wait3s'},
										autonomy={'next': Autonomy.Off, 'finish': Autonomy.Off},
										remapping={'input_value': 'no_site'})

			# x:86 y:142
			OperatableStateMachine.add('readSite',
										CalibrationSite(srdf_path='~/catkin_ws/dbparam/arm_waypoints.srdf', mirror=self.mirror, mirror_joint=[1,4,6]),
										transitions={'done': 'calibrationPosePlan'},
										autonomy={'done': Autonomy.Off},
										remapping={'arm_site': 'arm_site'})

			# x:1015 y:423
			OperatableStateMachine.add('shutApriltag',
										roslaunch_node_shut(),
										transitions={'done': 'calibration_16'},
										autonomy={'done': Autonomy.Off},
										remapping={'shutdown_class': 'shutdown_class2'})

			# x:1014 y:288
			OperatableStateMachine.add('shutCalibrate',
										roslaunch_node_shut(),
										transitions={'done': 'shutApriltag'},
										autonomy={'done': Autonomy.Off},
										remapping={'shutdown_class': 'shutdown_class1'})

			# x:691 y:571
			OperatableStateMachine.add('takeDec',
										DecisionState(outcomes=['True', 'False'], conditions=lambda x: 'True' if x else 'False'),
										transitions={'True': 'takeSample', 'False': 'TakeWeb'},
										autonomy={'True': Autonomy.Off, 'False': Autonomy.Off},
										remapping={'input_value': 'auto'})

			# x:861 y:422
			OperatableStateMachine.add('takeSample',
										comm_states__HandeyeAction(client_action='take', algorithm='Andreff', if_write=False),
										transitions={'done': 'helpClick'},
										autonomy={'done': Autonomy.Off},
										remapping={'sampler_in': 'sampler', 'sampler_out': 'sampler'})

			# x:412 y:55
			OperatableStateMachine.add('wait10s',
										WaitState(wait_time=10),
										transitions={'done': 'launchApriltag'},
										autonomy={'done': Autonomy.Off})

			# x:258 y:142
			OperatableStateMachine.add('wait18s',
										WaitState(wait_time=18),
										transitions={'done': 'readSite'},
										autonomy={'done': Autonomy.Off})

			# x:1032 y:57
			OperatableStateMachine.add('wait3s',
										WaitState(wait_time=3),
										transitions={'done': 'ComputeAndreff '},
										autonomy={'done': Autonomy.Off})

			# x:524 y:422
			OperatableStateMachine.add('wait6s',
										WaitState(wait_time=6),
										transitions={'done': 'firstDec'},
										autonomy={'done': Autonomy.Off})

			# x:1021 y:182
			OperatableStateMachine.add('ComputeAndreff ',
										comm_states__HandeyeAction(client_action='compute', algorithm='Andreff', if_write=True),
										transitions={'done': 'shutCalibrate'},
										autonomy={'done': Autonomy.Off},
										remapping={'sampler_in': 'sampler', 'sampler_out': 'sampler'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
