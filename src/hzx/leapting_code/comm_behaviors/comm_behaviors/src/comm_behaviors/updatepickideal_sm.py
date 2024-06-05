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
from comm_states.extracting_ideal_locations import ReadTfAndPublish
from comm_states.manipulation_share import ManipulationShare
from comm_states.publisherheader import PublishHeader
from comm_states.site_manipulation import SiteManipulation
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Mar 13 2024
@author: hzx
'''
class UpdatePickIdealSM(Behavior):
	'''
	UpdatePickIdeal
	'''


	def __init__(self):
		super(UpdatePickIdealSM, self).__init__()
		self.name = 'UpdatePickIdeal'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(StartPVMDetectSM, 'StartPVMDetect')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:641 y:61
		_state_machine = OperatableStateMachine(outcomes=['finished'], input_keys=['nav_goal'])
		_state_machine.userdata.nav_goal = {}

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('armInit',
										ManipulationShare(reference_frame='base_arm', end_effector_link='tool0'),
										transitions={'done': 'armPickupDetectPose'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:192 y:39
			OperatableStateMachine.add('armPickupDetectPose',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armPickupDetectPose", axis_value=["none",0], pos_targets=[], reference_frame="base_arm", end_effector_link="tool0", v_factor=1, a_factor=1, if_execute=True, wait_time=2, stay_level=False, cart_step_list=[3,11], retry_num=3, itp_norm=0.15, if_debug=False, cart_limit={}),
										transitions={'done': 'StartPVMDetect', 'failed': 'armPickupDetectPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:212 y:307
			OperatableStateMachine.add('readpickideal',
										ReadTfAndPublish(action="pick"),
										transitions={'success': 'wait5s', 'failed': 'readpickideal'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:400 y:185
			OperatableStateMachine.add('stopPickupDetect',
										PublishHeader(seq=0, frame_id="solar_detect"),
										transitions={'done': 'stopPickupSegment'},
										autonomy={'done': Autonomy.Off})

			# x:393 y:71
			OperatableStateMachine.add('stopPickupSegment',
										PublishHeader(seq=0, frame_id="enable_yolov8"),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:396 y:361
			OperatableStateMachine.add('wait5s',
										WaitState(wait_time=5),
										transitions={'done': 'stopPickupDetect'},
										autonomy={'done': Autonomy.Off})

			# x:208 y:198
			OperatableStateMachine.add('StartPVMDetect',
										self.use_behavior(StartPVMDetectSM, 'StartPVMDetect'),
										transitions={'finished': 'readpickideal'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'nav_goal': 'nav_goal'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
