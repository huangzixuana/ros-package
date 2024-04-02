#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_behaviors.cupon_sm import CupOnSM
from comm_behaviors.startpvmdetect_sm import StartPVMDetectSM
from comm_behaviors.stoppvmdetect_sm import StopPVMDetectSM
from comm_states.listen_solar import ListenSolar
from comm_states.manipulation_share import ManipulationShare
from comm_states.publisherheader import PublishHeader
from comm_states.realtime_goal import RealtimeGoal
from comm_states.site_manipulation import SiteManipulation
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jun 15 2023
@author: why
'''
class UninstallBracketPVMSM(Behavior):
	'''
	UninstallPVM
	'''


	def __init__(self):
		super(UninstallBracketPVMSM, self).__init__()
		self.name = 'UninstallBracketPVM'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(CupOnSM, 'CupOn')
		self.add_behavior(StartPVMDetectSM, 'StartPVMDetect')
		self.add_behavior(StopPVMDetectSM, 'StopPVMDetect')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:649 y:53
		_state_machine = OperatableStateMachine(outcomes=['finished'], output_keys=['nav_goal'])
		_state_machine.userdata.nav_goal = {'frame_id':'base_link', 'position':[0,0,0],'orientation':[0,0,0,1]}

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:60 y:23
			OperatableStateMachine.add('arminit',
										ManipulationShare(reference_frame='base_arm', end_effector_link='tool0'),
										transitions={'done': 'armTopPose'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:66 y:268
			OperatableStateMachine.add('StartPVMDetect',
										self.use_behavior(StartPVMDetectSM, 'StartPVMDetect'),
										transitions={'finished': 'checkUninstallSolar'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'nav_goal': 'nav_goal'})

			# x:368 y:391
			OperatableStateMachine.add('StopPVMDetect',
										self.use_behavior(StopPVMDetectSM, 'StopPVMDetect'),
										transitions={'finished': 'wait5s'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'nav_goal': 'nav_goal'})

			# x:577 y:468
			OperatableStateMachine.add('StopUninstallDetect',
										PublishHeader(seq=0, frame_id="solar_detect"),
										transitions={'done': 'CupOn'},
										autonomy={'done': Autonomy.Off})

			# x:570 y:629
			OperatableStateMachine.add('StopUninstallSegment',
										PublishHeader(seq=0, frame_id="enable_yolov8"),
										transitions={'done': 'StopUninstallDetect'},
										autonomy={'done': Autonomy.Off})

			# x:72 y:521
			OperatableStateMachine.add('WaitUninstallTarget',
										WaitState(wait_time=3),
										transitions={'done': 'realtimeGoal'},
										autonomy={'done': Autonomy.Off})

			# x:73 y:124
			OperatableStateMachine.add('armPlaceDetectPose',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armPlaceDetectPose", axis_value=["none",0], pos_targets=[], reference_frame="base_arm", end_effector_link="tool0", v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=False, cart_step_list=[3,11], retry_num=3, itp_norm=0.15, if_debug=False),
										transitions={'done': 'StartPVMDetect', 'failed': 'armPlaceDetectPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:571 y:167
			OperatableStateMachine.add('armPlaceDetectPose1',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armPlaceDetectPose", axis_value=["none",0], pos_targets=[], reference_frame="base_arm", end_effector_link="tool0", v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=False, cart_step_list=[3,11], retry_num=3, itp_norm=0.15, if_debug=False),
										transitions={'done': 'finished', 'failed': 'armPlaceDetectPose1'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:345 y:21
			OperatableStateMachine.add('armTopPose',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armTopPose", axis_value=["none",0], pos_targets=[], reference_frame="base_arm", end_effector_link="tool0", v_factor=1, a_factor=1, if_execute=True, wait_time=2, stay_level=False, cart_step_list=[3,11], retry_num=3, itp_norm=0.15, if_debug=False),
										transitions={'done': 'armPlaceDetectPose', 'failed': 'armTopPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:278 y:632
			OperatableStateMachine.add('armUninstallPVM-1',
										SiteManipulation(pos=[0,0,-0.01], quat=[0,0,0,1], target_frame="solar_link", target_name="none", axis_value=["none",0], pos_targets=[], reference_frame="base_arm", end_effector_link="tool0", v_factor=0.1, a_factor=0.1, if_execute=True, wait_time=5, stay_level=True, cart_step_list=[3,11], retry_num=3, itp_norm=0.1, if_debug=False),
										transitions={'done': 'StopUninstallSegment', 'failed': 'armUninstallPVM-1'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:72 y:397
			OperatableStateMachine.add('checkUninstallSolar',
										ListenSolar(timeout=60, fresh_time=3.0, ideal=[0.5, 2.3, 0, -0.04, -0.04, 1.65], tolerance=[0.5, 0.3, -1, 0.5, 0.5, 0.5]),
										transitions={'done': 'WaitUninstallTarget', 'timeout': 'StopPVMDetect'},
										autonomy={'done': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:83 y:627
			OperatableStateMachine.add('realtimeGoal',
										RealtimeGoal(position=[0,0,0], orientation=[0,0,0.7071,0.7071], frame_id='solar_link', source_frame='base_link', if_back=True, gap_pvm_pvm=-0.1, gap_pvm_chassis=0.2, chassis_width=2.2),
										transitions={'done': 'armUninstallPVM-1'},
										autonomy={'done': Autonomy.Off},
										remapping={'goal': 'nav_goal'})

			# x:392 y:276
			OperatableStateMachine.add('wait5s',
										WaitState(wait_time=5),
										transitions={'done': 'StartPVMDetect'},
										autonomy={'done': Autonomy.Off})

			# x:574 y:302
			OperatableStateMachine.add('CupOn',
										self.use_behavior(CupOnSM, 'CupOn'),
										transitions={'finished': 'armPlaceDetectPose1'},
										autonomy={'finished': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
