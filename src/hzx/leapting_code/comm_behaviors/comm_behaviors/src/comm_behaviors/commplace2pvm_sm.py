#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_behaviors.commpickuppvm_sm import CommPickupPVMSM
from comm_behaviors.cupoff_sm import CupOffSM
from comm_behaviors.placepvmdetect_sm import PlacePVMDetectSM
from comm_behaviors.secdetectpose_sm import secDetectPoseSM
from comm_behaviors.startpvmdetect_sm import StartPVMDetectSM
from comm_behaviors.stoppvmdetect_sm import StopPVMDetectSM
from comm_states.bracket_goal import BracketGoal
from comm_states.listen_solar import ListenSolar
from comm_states.manipulation_share import ManipulationShare
from comm_states.publisherheader import PublishHeader
from comm_states.realtime_goal import RealtimeGoal
from comm_states.site_manipulation import SiteManipulation
from comm_states.site_manipulation_ud import SiteManipulationUD
from comm_states.trig_decision import TrigDecision
from comm_states.update_tool0_pose import CalculateNewToolPose as comm_states__CalculateNewToolPose
from flexbe_states.decision_state import DecisionState
from flexbe_states.log_state import LogState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jun 05 2023
@author: hzx
'''
class CommPlace2PVMSM(Behavior):
	'''
	PlacePVM
	'''


	def __init__(self):
		super(CommPlace2PVMSM, self).__init__()
		self.name = 'CommPlace2PVM'

		# parameters of this behavior
		self.add_parameter('mode', 'SimPlacePVM')
		self.add_parameter('pvm_width', 1134)
		self.add_parameter('install_gap', -40)
		self.add_parameter('place_solar_x', -0.03)
		self.add_parameter('place_solar_z', 0.13)
		self.add_parameter('place_ideal_x', 0.6)
		self.add_parameter('place_ideal_y', 2.4)
		self.add_parameter('place_ideal_z', 0.001)
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
		self.add_parameter('sec_offset_x', 0.0)
		self.add_parameter('sec_offset_y', 0.0)
		self.add_parameter('sec_offset_z', 0.0)

		# references to used behaviors
		self.add_behavior(CommPickupPVMSM, 'CommPickupPVM')
		self.add_behavior(CupOffSM, 'CupOff')
		self.add_behavior(PlacePVMDetectSM, 'PlacePVMDetect')
		self.add_behavior(PlacePVMDetectSM, 'PlacePVMDetect_2')
		self.add_behavior(StartPVMDetectSM, 'StartPVMDetect')
		self.add_behavior(StopPVMDetectSM, 'StopPVMDetect')
		self.add_behavior(StopPVMDetectSM, 'StopPVMDetect2')
		self.add_behavior(secDetectPoseSM, 'secDetectPose')
		self.add_behavior(secDetectPoseSM, 'secDetectPose_2')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:880 y:819
		_state_machine = OperatableStateMachine(outcomes=['finished'], input_keys=['pose_msg_in', 'pvm_num', 'robot_pose', 'tool_pose'], output_keys=['nav_goal'])
		_state_machine.userdata.nav_goal = {}
		_state_machine.userdata.mode = self.mode
		_state_machine.userdata.empty_goal = {}
		_state_machine.userdata.detect_option = self.detect_option
		_state_machine.userdata.pose_msg_in = None
		_state_machine.userdata.robot_pose = None
		_state_machine.userdata.tool_pose = None
		_state_machine.userdata.pvm_num = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:68 y:59
			OperatableStateMachine.add('armInit',
										ManipulationShare(reference_frame='base_arm', end_effector_link='tool0'),
										transitions={'done': 'armPlace2DetectPose'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:629 y:667
			OperatableStateMachine.add('CupOff',
										self.use_behavior(CupOffSM, 'CupOff'),
										transitions={'finished': 'storeToolpose'},
										autonomy={'finished': Autonomy.Inherit})

			# x:432 y:565
			OperatableStateMachine.add('PlacePVMDetect',
										self.use_behavior(PlacePVMDetectSM, 'PlacePVMDetect',
											parameters={'place_solar_x': self.place_solar_x, 'pvm_width': self.pvm_width, 'install_gap': self.install_gap, 'place_solar_z': self.place_solar_z, 'en_bracket': False, 'en_plane': True, 'yline_gap': self.line_gap, 'line_z': self.line_z}),
										transitions={'finished': 'CupOff'},
										autonomy={'finished': Autonomy.Inherit})

			# x:823 y:710
			OperatableStateMachine.add('PlacePVMDetect_2',
										self.use_behavior(PlacePVMDetectSM, 'PlacePVMDetect_2',
											parameters={'place_solar_x': self.place_solar_x, 'pvm_width': self.pvm_width*2, 'install_gap': self.install_gap*2+20, 'place_solar_z': self.place_solar_z, 'en_bracket': False, 'en_plane': True, 'yline_gap': self.line_gap, 'line_z': self.line_z}),
										transitions={'finished': 'finished'},
										autonomy={'finished': Autonomy.Inherit})

			# x:439 y:187
			OperatableStateMachine.add('StartPVMDetect',
										self.use_behavior(StartPVMDetectSM, 'StartPVMDetect'),
										transitions={'finished': 'detectSucceed'},
										autonomy={'finished': Autonomy.Inherit})

			# x:881 y:191
			OperatableStateMachine.add('StopPVMDetect',
										self.use_behavior(StopPVMDetectSM, 'StopPVMDetect'),
										transitions={'finished': 'wait5s'},
										autonomy={'finished': Autonomy.Inherit})

			# x:227 y:670
			OperatableStateMachine.add('StopPVMDetect2',
										self.use_behavior(StopPVMDetectSM, 'StopPVMDetect2'),
										transitions={'finished': 'detectDec'},
										autonomy={'finished': Autonomy.Inherit})

			# x:222 y:59
			OperatableStateMachine.add('armPlace2DetectPose',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armPlace2DetectPose", axis_value=["none",0], pos_targets=[], trajectory_name='none', reference_frame="base_arm", end_effector_link="tool0", wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=False, cart_step_list=[3,11], step_factor=0.1, itp_norm=0.15, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id='none', plan_time=2),
										transitions={'done': 'compare', 'failed': 'armPlace2DetectPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:624 y:396
			OperatableStateMachine.add('armPlace2DetectPose-2',
										SiteManipulation(pos=[0, 0, 0], quat=[0, 0, 0, 1], target_frame='none', target_name="armPlace2DetectPose", axis_value=['none', 0], pos_targets=[], trajectory_name='none', reference_frame='base_arm', end_effector_link='tool0', wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=False, cart_step_list=[3, 11], step_factor=0.1, itp_norm=0.15, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id='none', plan_time=2),
										transitions={'done': 'armTopPose', 'failed': 'armPlace2DetectPose-2'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:825 y:562
			OperatableStateMachine.add('armPlace2DetectPose2',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armPlace2DetectPose", axis_value=["none",0], pos_targets=[], trajectory_name="none", reference_frame="base_arm", end_effector_link="tool0", wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=False, cart_step_list=[3,11], step_factor=0.1, itp_norm=0.15, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id="none", plan_time=2),
										transitions={'done': 'secDetectPose_2', 'failed': 'armPlace2DetectPose2'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:438 y:663
			OperatableStateMachine.add('armPlaceLineUD',
										SiteManipulationUD(reference_frame="base_arm", end_effector_link="tool0", v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=True, cart_step_list=[3,11], step_factor=0.1, retry_num=3, itp_norm=0.15, if_debug=False, cart_limit={}),
										transitions={'done': 'PlacePVMDetect', 'failed': 'armPlaceLineUD'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'mani_goal': 'line_goal'})

			# x:837 y:396
			OperatableStateMachine.add('armTopPose',
										SiteManipulation(pos=[0, 0, 0], quat=[0, 0, 0, 1], target_frame='none', target_name="armTopPose", axis_value=['none', 0], pos_targets=[], trajectory_name='none', reference_frame='base_arm', end_effector_link='tool0', wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=False, cart_step_list=[3, 11], step_factor=0.1, itp_norm=0.15, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id='none', plan_time=2),
										transitions={'done': 'CommPickupPVM', 'failed': 'armTopPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:457 y:400
			OperatableStateMachine.add('armUp10cm',
										SiteManipulation(pos=[0,0,0.1], quat=[0, 0, 0, 1], target_frame='tool0', target_name='none', axis_value=['none', 0], pos_targets=[], trajectory_name='none', reference_frame='base_arm', end_effector_link='tool0', wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=True, cart_step_list=[1, 11], step_factor=0.1, itp_norm=0, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id='none', plan_time=2),
										transitions={'done': 'armPlace2DetectPose-2', 'failed': 'armUp10cm'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:453 y:295
			OperatableStateMachine.add('checkPlaceSolar',
										ListenSolar(timeout=60, fresh_time=3.0, ideal=[self.place_ideal_x, self.place_ideal_y, self.place_ideal_z, self.place_ideal_er, self.place_ideal_ep, self.place_ideal_ey], tolerance=[self.place_tol_x, self.place_tol_y, self.place_tol_z, self.place_tol_er, self.place_tol_ep, self.place_ideal_ey], pose_topic='filter_solar_pose'),
										transitions={'done': 'log2', 'timeout': 'detectFailed'},
										autonomy={'done': Autonomy.Off, 'timeout': Autonomy.Off},
										remapping={'pose_msg': 'pose_msg'})

			# x:570 y:58
			OperatableStateMachine.add('compare',
										DecisionState(outcomes=['fri','sec'], conditions=lambda x: 'fri'  if x<2 else 'sec'),
										transitions={'fri': 'waitCamDelay', 'sec': 'secDetectPose'},
										autonomy={'fri': Autonomy.Off, 'sec': Autonomy.Off},
										remapping={'input_value': 'pvm_num'})

			# x:907 y:294
			OperatableStateMachine.add('dec',
										TrigDecision(),
										transitions={'next': 'StopPVMDetect', 'withdraw': 'dec'},
										autonomy={'next': Autonomy.Off, 'withdraw': Autonomy.Off})

			# x:241 y:779
			OperatableStateMachine.add('detectDec',
										DecisionState(outcomes=['line', 'bkt', 'pvm', 'invalid'], conditions=lambda x: x if (x in ['line', 'bkt', 'pvm']) else 'invalid'),
										transitions={'line': 'solarLinkInfo', 'bkt': 'solarLinkInfo', 'pvm': 'solarLinkInfo', 'invalid': 'detectDec'},
										autonomy={'line': Autonomy.Off, 'bkt': Autonomy.Off, 'pvm': Autonomy.Off, 'invalid': Autonomy.Off},
										remapping={'input_value': 'detect_option'})

			# x:701 y:296
			OperatableStateMachine.add('detectFailed',
										PublishHeader(seq=0, frame_id='install_detect_put'),
										transitions={'done': 'dec'},
										autonomy={'done': Autonomy.Off})

			# x:216 y:188
			OperatableStateMachine.add('detectSucceed',
										PublishHeader(seq=1, frame_id="install_detect_put"),
										transitions={'done': 'waitTargetStable2s'},
										autonomy={'done': Autonomy.Off})

			# x:242 y:559
			OperatableStateMachine.add('log2',
										LogState(text='place', severity=Logger.REPORT_HINT),
										transitions={'done': 'StopPVMDetect2'},
										autonomy={'done': Autonomy.Off})

			# x:75 y:495
			OperatableStateMachine.add('modeDecision',
										DecisionState(outcomes=['PlacePVM' ,'SimPlacePVM','invalid'], conditions=lambda x: x if (x in ['PlacePVM' ,'SimPlacePVM']) else 'invalid'),
										transitions={'PlacePVM': 'realtimeGoal*', 'SimPlacePVM': 'log2', 'invalid': 'modeDecision'},
										autonomy={'PlacePVM': Autonomy.Off, 'SimPlacePVM': Autonomy.Off, 'invalid': Autonomy.Off},
										remapping={'input_value': 'mode'})

			# x:74 y:610
			OperatableStateMachine.add('realtimeGoal*',
										RealtimeGoal(position=[1.0,0.75,0], orientation=[0, 0, 0.7071067, 0.7071067], frame_id='solar_link', source_frame='base_link', if_back=False, gap_pvm_pvm=1.6, gap_pvm_chassis=0.4, chassis_width=2.2, k_y=0.2, k_yaw=0.2, use_solar=True),
										transitions={'done': 'log2'},
										autonomy={'done': Autonomy.Off},
										remapping={'goal': 'nav_goal'})

			# x:634 y:492
			OperatableStateMachine.add('realtimeGoalTool0',
										RealtimeGoal(position=[1.0,0.75,0], orientation=[0, 0, 0.7071067, 0.7071067], frame_id='solar_link', source_frame='base_link', if_back=False, gap_pvm_pvm=0.4, gap_pvm_chassis=0.4, chassis_width=2.2, k_y=0.2, k_yaw=0.2, use_solar=False),
										transitions={'done': 'armUp10cm'},
										autonomy={'done': Autonomy.Off},
										remapping={'goal': 'nav_goal'})

			# x:738 y:52
			OperatableStateMachine.add('secDetectPose',
										self.use_behavior(secDetectPoseSM, 'secDetectPose',
											parameters={'sec_offset_x': self.sec_offset_x, 'sec_offset_y': self.sec_offset_y, 'sec_offset_z': self.sec_offset_z}),
										transitions={'finished': 'PlacePVMDetect'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'robot_pose': 'robot_pose', 'tool_pose': 'tool_pose'})

			# x:824 y:635
			OperatableStateMachine.add('secDetectPose_2',
										self.use_behavior(secDetectPoseSM, 'secDetectPose_2',
											parameters={'sec_offset_x': self.sec_offset_x, 'sec_offset_y': self.sec_offset_y, 'sec_offset_z': self.sec_offset_z}),
										transitions={'finished': 'PlacePVMDetect_2'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'robot_pose': 'robot_pose', 'tool_pose': 'tool_pose'})

			# x:448 y:782
			OperatableStateMachine.add('solarLinkInfo',
										BracketGoal(pos=[self.place_solar_x, (self.pvm_width+ self.install_gap)*0.001+0.1, self.place_solar_z+0.05], quat=[0,0,0,1], dx_max=0.1, limit_dict={}, pos_targets=[], itp_norm=0.0),
										transitions={'done': 'armPlaceLineUD'},
										autonomy={'done': Autonomy.Off},
										remapping={'pose_msg': 'pose_msg', 'mani_goal': 'line_goal'})

			# x:627 y:579
			OperatableStateMachine.add('storeToolpose',
										comm_states__CalculateNewToolPose(action="pose_in", offset_x=-1.2, offset_y=0.0, offset_z=0.12),
										transitions={'done': 'realtimeGoalTool0'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group', 'robot_pose': 'robot_pose', 'tool_pose': 'tool_pose'})

			# x:697 y:189
			OperatableStateMachine.add('wait5s',
										WaitState(wait_time=5),
										transitions={'done': 'StartPVMDetect'},
										autonomy={'done': Autonomy.Off})

			# x:439 y:109
			OperatableStateMachine.add('waitCamDelay',
										WaitState(wait_time=2),
										transitions={'done': 'StartPVMDetect'},
										autonomy={'done': Autonomy.Off})

			# x:210 y:294
			OperatableStateMachine.add('waitTargetStable2s',
										WaitState(wait_time=2),
										transitions={'done': 'checkPlaceSolar'},
										autonomy={'done': Autonomy.Off})

			# x:817 y:477
			OperatableStateMachine.add('CommPickupPVM',
										self.use_behavior(CommPickupPVMSM, 'CommPickupPVM',
											parameters={'mode': "StaticPickupPVM", 'detect_option': "sec"}),
										transitions={'finished': 'armPlace2DetectPose2'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'nav_goal': 'nav_goal', 'pose_msg_in': 'pose_msg_in', 'robot_pose': 'robot_pose', 'tool_pose': 'tool_pose', 'pose_msg_pick': 'pose_msg_pick'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
