#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_behaviors.change_bz_sm import change_bzSM
from comm_behaviors.cupon_sm import CupOnSM
from comm_behaviors.startpvmdetect_sm import StartPVMDetectSM
from comm_behaviors.stoppvmdetect_sm import StopPVMDetectSM
from comm_states.listen_solar import ListenSolar
from comm_states.manipulation_share import ManipulationShare
from comm_states.publisherheader import PublishHeader
from comm_states.realtime_goal import RealtimeGoal
from comm_states.site_manipulation import SiteManipulation
from comm_states.site_navigation import SiteNavigation
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jun 15 2023
@author: ZCX
'''
class UninstallnavtestSM(Behavior):
	'''
	UninstallPVM
	'''


	def __init__(self):
		super(UninstallnavtestSM, self).__init__()
		self.name = 'Uninstall-nav-test'

		# parameters of this behavior
		self.add_parameter('unin_solar_x', 0.0001)
		self.add_parameter('unin_solar_y', 0.0001)
		self.add_parameter('unin_solar_z', 0.08)
		self.add_parameter('if_nav_back', True)

		# references to used behaviors
		self.add_behavior(CupOnSM, 'CupOn')
		self.add_behavior(StartPVMDetectSM, 'StartPVMDetect')
		self.add_behavior(StopPVMDetectSM, 'StopPVMDetect')
		self.add_behavior(StopPVMDetectSM, 'StopPVMDetect_2')
		self.add_behavior(change_bzSM, 'change_bz')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:649 y:41, x:625 y:826
		_state_machine = OperatableStateMachine(outcomes=['finished', 'o2'], output_keys=['nav_goal'])
		_state_machine.userdata.nav_goal = {'frame_id':'base_link', 'position':[0,0,0],'orientation':[0,0,0,1]}

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:51 y:21
			OperatableStateMachine.add('change_bz',
										self.use_behavior(change_bzSM, 'change_bz'),
										transitions={'finished': 'armInit'},
										autonomy={'finished': Autonomy.Inherit})

			# x:77 y:292
			OperatableStateMachine.add('StartPVMDetect',
										self.use_behavior(StartPVMDetectSM, 'StartPVMDetect'),
										transitions={'finished': 'waitUninstallTarget2s'},
										autonomy={'finished': Autonomy.Inherit})

			# x:306 y:516
			OperatableStateMachine.add('StopPVMDetect',
										self.use_behavior(StopPVMDetectSM, 'StopPVMDetect'),
										transitions={'finished': 'wait5s'},
										autonomy={'finished': Autonomy.Inherit})

			# x:202 y:698
			OperatableStateMachine.add('StopPVMDetect_2',
										self.use_behavior(StopPVMDetectSM, 'StopPVMDetect_2'),
										transitions={'finished': 'nav'},
										autonomy={'finished': Autonomy.Inherit})

			# x:314 y:27
			OperatableStateMachine.add('armInit',
										ManipulationShare(reference_frame='base_arm', end_effector_link='tool0'),
										transitions={'done': 'armPlaceDetectPose'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:82 y:114
			OperatableStateMachine.add('armPlaceDetectPose',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armPlaceDetectPose", axis_value=["none",0], pos_targets=[], trajectory_name='none', reference_frame="base_arm", end_effector_link="tool0", wait_time=2, v_factor=1, a_factor=1, t_factor=1.0, stay_level=False, cart_step_list=[3,11], step_factor=0.1, itp_norm=0.15, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id='none', plan_time=2),
										transitions={'done': 'wait2s', 'failed': 'armPlaceDetectPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:568 y:114
			OperatableStateMachine.add('armPlaceDetectPose1',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armPlaceDetectPose", axis_value=["none",0], pos_targets=[], trajectory_name='none', reference_frame="base_arm", end_effector_link="tool0", wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=False, cart_step_list=[3,11], step_factor=0.1, itp_norm=0.15, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id='none', plan_time=2),
										transitions={'done': 'finished', 'failed': 'armPlaceDetectPose1'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:316 y:111
			OperatableStateMachine.add('armTopPose',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armTopPose", axis_value=["none",0], pos_targets=[], trajectory_name='none', reference_frame="base_arm", end_effector_link="tool0", wait_time=2, v_factor=1, a_factor=1, t_factor=1.0, stay_level=False, cart_step_list=[3,11], step_factor=0.1, itp_norm=0.15, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id='none', plan_time=2),
										transitions={'done': 'armPlaceDetectPose', 'failed': 'armTopPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:295 y:626
			OperatableStateMachine.add('armUninstallPVM',
										SiteManipulation(pos=[self.unin_solar_x, self.unin_solar_y, self.unin_solar_z], quat=[0,0,0,1], target_frame="solar_link", target_name="none", axis_value=["none",0], pos_targets=[], trajectory_name='none', reference_frame="base_arm", end_effector_link="tool0", wait_time=5, v_factor=1, a_factor=1, t_factor=1.0, stay_level=True, cart_step_list=[1,15], step_factor=0.1, itp_norm=0.1, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id='none', plan_time=2),
										transitions={'done': 'stopUninstallSegment', 'failed': 'armUninstallPVM'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:583 y:207
			OperatableStateMachine.add('armUp10cm',
										SiteManipulation(pos=[0, 0, 0.1], quat=[0, 0, 0, 1], target_frame='tool0', target_name='none', axis_value=['none', 0], pos_targets=[], trajectory_name='none', reference_frame='base_arm', end_effector_link='tool0', wait_time=0.5, v_factor=1, a_factor=1, t_factor=1.0, stay_level=True, cart_step_list=[1, 11], step_factor=0.1, itp_norm=0, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id='none', plan_time=2),
										transitions={'done': 'armPlaceDetectPose1', 'failed': 'armUp10cm'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:81 y:521
			OperatableStateMachine.add('checkUninstallSolar',
										ListenSolar(timeout=60, fresh_time=3.0, ideal=[0.5, 2.3, 0, -0.04, -0.04, 1.65], tolerance=[0.7, 0.5, -1, 0.5, 0.5, 0.5], pose_topic='filter_solar_pose'),
										transitions={'done': 'realtimeGoal', 'timeout': 'StopPVMDetect'},
										autonomy={'done': Autonomy.Off, 'timeout': Autonomy.Off},
										remapping={'pose_msg': 'pose_msg'})

			# x:401 y:738
			OperatableStateMachine.add('nav',
										SiteNavigation(site_name="", position=[0,0,0], orientation=[0,0,0,1], frame_id="map", base_link2map=True),
										transitions={'arrived': 'o2', 'canceled': 'nav', 'failed': 'nav'},
										autonomy={'arrived': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'nav_goal': 'nav_goal'})

			# x:91 y:626
			OperatableStateMachine.add('realtimeGoal',
										RealtimeGoal(position=[0,0,0], orientation=[0,0,0.7071,0.7071], frame_id='solar_link', source_frame='base_link', if_back=self.if_nav_back, gap_pvm_pvm=-0.2, gap_pvm_chassis=0.25, chassis_width=2.2, k_y=0, k_yaw=0),
										transitions={'done': 'StopPVMDetect_2'},
										autonomy={'done': Autonomy.Off},
										remapping={'goal': 'nav_goal'})

			# x:577 y:468
			OperatableStateMachine.add('stopUninstallDetect',
										PublishHeader(seq=0, frame_id="solar_detect"),
										transitions={'done': 'CupOn'},
										autonomy={'done': Autonomy.Off})

			# x:574 y:623
			OperatableStateMachine.add('stopUninstallSegment',
										PublishHeader(seq=0, frame_id="enable_yolov8"),
										transitions={'done': 'stopUninstallDetect'},
										autonomy={'done': Autonomy.Off})

			# x:91 y:199
			OperatableStateMachine.add('wait2s',
										WaitState(wait_time=2),
										transitions={'done': 'StartPVMDetect'},
										autonomy={'done': Autonomy.Off})

			# x:322 y:295
			OperatableStateMachine.add('wait5s',
										WaitState(wait_time=5),
										transitions={'done': 'StartPVMDetect'},
										autonomy={'done': Autonomy.Off})

			# x:76 y:408
			OperatableStateMachine.add('waitUninstallTarget2s',
										WaitState(wait_time=2),
										transitions={'done': 'checkUninstallSolar'},
										autonomy={'done': Autonomy.Off})

			# x:574 y:302
			OperatableStateMachine.add('CupOn',
										self.use_behavior(CupOnSM, 'CupOn'),
										transitions={'finished': 'armUp10cm'},
										autonomy={'finished': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
