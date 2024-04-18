#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_behaviors.fromplace2pick_sm import fromPlace2PickSM
from comm_states.listen_state import ListenState
from comm_states.manipulation_share import ManipulationShare
from comm_states.publisherheader import PublishHeader
from comm_states.site_manipulation import SiteManipulation
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Sep 19 2023
@author: Lei Zeng
'''
class StartInstallCheckSM(Behavior):
	'''
	check start
	'''


	def __init__(self):
		super(StartInstallCheckSM, self).__init__()
		self.name = 'StartInstallCheck'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(fromPlace2PickSM, 'fromPlace2Pick')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:381 y:384
		_state_machine = OperatableStateMachine(outcomes=['finished'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:66 y:71
			OperatableStateMachine.add('armIniit',
										ManipulationShare(reference_frame="base_arm", end_effector_link="tool0"),
										transitions={'done': 'armTopPose'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:463 y:75
			OperatableStateMachine.add('armPlaceDetectPose',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armPlaceDetectPose", axis_value=["none",0], pos_targets=[], reference_frame="base_arm", end_effector_link="tool0", v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=False, cart_step_list=[3,11], retry_num=3, itp_norm=0.15, if_debug=False),
										transitions={'done': 'startSegment', 'failed': 'armPlaceDetectPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:245 y:73
			OperatableStateMachine.add('armTopPose',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armTopPose", axis_value=["none",0], pos_targets=[], reference_frame="base_arm", end_effector_link="tool0", v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=False, cart_step_list=[3,11], retry_num=3, itp_norm=0.15, if_debug=False),
										transitions={'done': 'armPlaceDetectPose', 'failed': 'armTopPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:543 y:200
			OperatableStateMachine.add('checkTF',
										ListenState(target_frame='solar_link', source_frame='base_arm', timeout=60, fresh_time=3.0, ideal=[0, 0, 0, 0, 0, 0], tolerance=[-1, -1, -1, -1, -1, -1]),
										transitions={'done': 'detectSucceed', 'timeout': 'detectFailed'},
										autonomy={'done': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:541 y:289
			OperatableStateMachine.add('detectFailed',
										PublishHeader(seq=0, frame_id="install_detect_put"),
										transitions={'done': 'checkTF'},
										autonomy={'done': Autonomy.Off})

			# x:374 y:198
			OperatableStateMachine.add('detectSucceed',
										PublishHeader(seq=1, frame_id="install_detect_put"),
										transitions={'done': 'stopSegment'},
										autonomy={'done': Autonomy.Off})

			# x:54 y:370
			OperatableStateMachine.add('fromPlace2Pick',
										self.use_behavior(fromPlace2PickSM, 'fromPlace2Pick'),
										transitions={'finished': 'finished'},
										autonomy={'finished': Autonomy.Inherit})

			# x:724 y:199
			OperatableStateMachine.add('startDetect',
										PublishHeader(seq=2, frame_id="solar_detect"),
										transitions={'done': 'checkTF'},
										autonomy={'done': Autonomy.Off})

			# x:722 y:72
			OperatableStateMachine.add('startSegment',
										PublishHeader(seq=2, frame_id="enable_yolov8"),
										transitions={'done': 'startDetect'},
										autonomy={'done': Autonomy.Off})

			# x:67 y:204
			OperatableStateMachine.add('stopDetect',
										PublishHeader(seq=0, frame_id="solar_detect"),
										transitions={'done': 'fromPlace2Pick'},
										autonomy={'done': Autonomy.Off})

			# x:243 y:202
			OperatableStateMachine.add('stopSegment',
										PublishHeader(seq=0, frame_id="enable_yolov8"),
										transitions={'done': 'stopDetect'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]