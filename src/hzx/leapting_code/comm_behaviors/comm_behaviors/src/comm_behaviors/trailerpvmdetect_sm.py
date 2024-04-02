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
from comm_states.site_manipulation import SiteManipulation
from comm_states.trig_decision import TrigDecision
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
		self.add_parameter('pick_solar_x', 0)
		self.add_parameter('pick_solar_y', 0)
		self.add_parameter('pick_solar_z', -0.01)
		self.add_parameter('pick_ideal_x', 2.1)
		self.add_parameter('pick_ideal_y', -0.006)
		self.add_parameter('pick_ideal_z', 0.0)
		self.add_parameter('pick_ideal_er', 0.006)
		self.add_parameter('pick_ideal_ep', -0.011)
		self.add_parameter('pick_ideal_ey', 1.559)
		self.add_parameter('pick_tol_x', 0.05)
		self.add_parameter('pick_tol_y', 0.1)
		self.add_parameter('pick_tol_z', -1.0)
		self.add_parameter('pick_tol_er', 0.2)
		self.add_parameter('pick_tol_ep', 0.2)
		self.add_parameter('pick_tol_ey', 0.2)

		# references to used behaviors
		self.add_behavior(StartPVMDetectSM, 'StartPVMDetect')
		self.add_behavior(StopPVMDetectSM, 'StopPVMDetect')

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

			# x:421 y:569
			OperatableStateMachine.add('DetectSucceed',
										PublishHeader(seq=1, frame_id="install_detect_pick"),
										transitions={'done': 'waitTargetStable5s'},
										autonomy={'done': Autonomy.Off})

			# x:208 y:198
			OperatableStateMachine.add('StartPVMDetect',
										self.use_behavior(StartPVMDetectSM, 'StartPVMDetect'),
										transitions={'finished': 'checkPickupSolar'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'nav_goal': 'nav_goal'})

			# x:7 y:214
			OperatableStateMachine.add('StopPVMDetect',
										self.use_behavior(StopPVMDetectSM, 'StopPVMDetect'),
										transitions={'finished': 'wait5s'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'nav_goal': 'nav_goal'})

			# x:192 y:39
			OperatableStateMachine.add('armPickupDetectPose',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armPickupDetectPose", axis_value=["none",0], pos_targets=[], reference_frame="base_arm", end_effector_link="tool0", v_factor=1, a_factor=1, if_execute=True, wait_time=2, stay_level=False, cart_step_list=[3,11], retry_num=3, itp_norm=0.15, if_debug=False),
										transitions={'done': 'StartPVMDetect', 'failed': 'armPickupDetectPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:399 y:277
			OperatableStateMachine.add('armPickupPVM',
										SiteManipulation(pos=[self.pick_solar_x, self.pick_solar_y, self.pick_solar_z], quat=[0, 0, 0, 1], target_frame="solar_link", target_name="none", axis_value=["none",0], pos_targets=[], reference_frame="base_arm", end_effector_link="tool0", v_factor=0.1, a_factor=0.1, if_execute=True, wait_time=0.5, stay_level=True, cart_step_list=[3,7], retry_num=5, itp_norm=0.1, if_debug=True),
										transitions={'done': 'stopPickupDetect', 'failed': 'wait3s'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:212 y:367
			OperatableStateMachine.add('checkPickupSolar',
										ListenSolar(timeout=60, fresh_time=3.0, ideal=[self.pick_ideal_x, self.pick_ideal_y, self.pick_ideal_z, self.pick_ideal_er, self.pick_ideal_ep, self.pick_ideal_ey], tolerance=[self.pick_tol_x, self.pick_tol_y, self.pick_tol_z, self.pick_tol_er, self.pick_tol_ep, self.pick_tol_ey]),
										transitions={'done': 'DetectSucceed', 'timeout': 'DetectFailed'},
										autonomy={'done': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:44 y:338
			OperatableStateMachine.add('dec1',
										TrigDecision(),
										transitions={'next': 'StopPVMDetect', 'withdraw': 'dec1'},
										autonomy={'next': Autonomy.Off, 'withdraw': Autonomy.Off})

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

			# x:603 y:279
			OperatableStateMachine.add('wait3s',
										WaitState(wait_time=3),
										transitions={'done': 'armPickupPVM'},
										autonomy={'done': Autonomy.Off})

			# x:29 y:122
			OperatableStateMachine.add('wait5s',
										WaitState(wait_time=5),
										transitions={'done': 'StartPVMDetect'},
										autonomy={'done': Autonomy.Off})

			# x:30 y:487
			OperatableStateMachine.add('waitPickupTarget',
										WaitState(wait_time=3),
										transitions={'done': 'dec1'},
										autonomy={'done': Autonomy.Off})

			# x:396 y:361
			OperatableStateMachine.add('waitTargetStable5s',
										WaitState(wait_time=2),
										transitions={'done': 'armPickupPVM'},
										autonomy={'done': Autonomy.Off})

			# x:28 y:571
			OperatableStateMachine.add('DetectFailed',
										PublishHeader(seq=0, frame_id="install_detect_pick"),
										transitions={'done': 'waitPickupTarget'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
