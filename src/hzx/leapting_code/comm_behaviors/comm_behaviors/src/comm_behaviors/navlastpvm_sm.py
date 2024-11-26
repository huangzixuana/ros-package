#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_behaviors.cupoff_sm import CupOffSM
from comm_behaviors.fromplace2pick_sm import fromPlace2PickSM
from comm_states.listen_solar import ListenSolar
from comm_states.manipulation_share import ManipulationShare
from comm_states.publisherheader import PublishHeader
from comm_states.site_manipulation import SiteManipulation
from comm_states.site_navigation import SiteNavigation
from flexbe_states.decision_state import DecisionState
from flexbe_states.subscriber_state import SubscriberState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Feb 25 2024
@author: zcx
'''
class NavLastPVMSM(Behavior):
	'''
	nav or not after installing the last pvm
	'''


	def __init__(self):
		super(NavLastPVMSM, self).__init__()
		self.name = 'NavLastPVM'

		# parameters of this behavior
		self.add_parameter('pick_solar_x', 0.0)
		self.add_parameter('pick_solar_y', 0.0)
		self.add_parameter('pick_solar_z', 0.0)

		# references to used behaviors
		self.add_behavior(CupOffSM, 'CupOff')
		self.add_behavior(fromPlace2PickSM, 'FromPlace2Pick')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:998 y:156
		_state_machine = OperatableStateMachine(outcomes=['finished'], input_keys=['nav_goal'])
		_state_machine.userdata.nav_goal = {'frame_id':'base_link', 'position':[0,0,0],'orientation':[0,0,0,1]}

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:95 y:24
			OperatableStateMachine.add('armInit',
										ManipulationShare(reference_frame="base_arm", end_effector_link="tool0"),
										transitions={'done': 'CupOff'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:435 y:20
			OperatableStateMachine.add('FromPlace2Pick',
										self.use_behavior(fromPlace2PickSM, 'FromPlace2Pick'),
										transitions={'finished': 'startSegment'},
										autonomy={'finished': Autonomy.Inherit})

			# x:778 y:18
			OperatableStateMachine.add('UI:go?stop?',
										SubscriberState(topic="trig", blocking=True, clear=True),
										transitions={'received': 'navDecision', 'unavailable': 'UI:go?stop?'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'msg'})

			# x:944 y:287
			OperatableStateMachine.add('armPickupDetectPose',
										SiteManipulation(pos=[0, 0, 0], quat=[0, 0, 0, 1], target_frame='none', target_name='armPickupDetectPose', axis_value=['none', 0], pos_targets=[], trajectory_name="none", reference_frame='base_arm', end_effector_link='tool0', wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=False, cart_step_list=[3, 11], step_factor=0.1, itp_norm=0.15, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id='none', plan_time=2),
										transitions={'done': 'finished', 'failed': 'armPickupDetectPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:102 y:496
			OperatableStateMachine.add('armPickupPVM',
										SiteManipulation(pos=[self.pick_solar_x, self.pick_solar_y, self.pick_solar_z], quat=[0,0,0,1], target_frame="solar_link", target_name="none", axis_value=["none",0], pos_targets=[], trajectory_name="none", reference_frame="base_arm", end_effector_link="tool0", wait_time=0.5, v_factor=1, a_factor=1, t_factor=1.0, stay_level=True, cart_step_list=[3,7], step_factor=0.1, itp_norm=0.1, retry_num=5, cart_limit={}, if_execute=True, if_debug=True, planner_id='none', plan_time=2),
										transitions={'done': 'wait5s', 'failed': 'wait2s'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:965 y:492
			OperatableStateMachine.add('baseBackward',
										SiteNavigation(site_name="", position=[0,0,0], orientation=[0,0,0,1], frame_id="base_link", base_link2map=True),
										transitions={'arrived': 'armPickupDetectPose', 'canceled': 'baseBackward', 'failed': 'baseBackward'},
										autonomy={'arrived': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'nav_goal': 'nav_goal'})

			# x:450 y:272
			OperatableStateMachine.add('detectFailed',
										PublishHeader(seq=0, frame_id="install_detect_put"),
										transitions={'done': 'stopDetect'},
										autonomy={'done': Autonomy.Off})

			# x:102 y:385
			OperatableStateMachine.add('detectSucceed',
										PublishHeader(seq=2, frame_id="install_detect_put"),
										transitions={'done': 'armPickupPVM'},
										autonomy={'done': Autonomy.Off})

			# x:620 y:21
			OperatableStateMachine.add('informWeb',
										PublishHeader(seq=2, frame_id="UI_continue"),
										transitions={'done': 'UI:go?stop?'},
										autonomy={'done': Autonomy.Off})

			# x:107 y:273
			OperatableStateMachine.add('listenSolar',
										ListenSolar(timeout=60, fresh_time=3.0, ideal=[0,0,0,0,0,0], tolerance=[-1,-1,-1,-1,-1,-1], pose_topic="filter_solar_pose"),
										transitions={'done': 'detectSucceed', 'timeout': 'detectFailed'},
										autonomy={'done': Autonomy.Off, 'timeout': Autonomy.Off},
										remapping={'pose_msg': 'pose_msg'})

			# x:784 y:151
			OperatableStateMachine.add('navDecision',
										DecisionState(outcomes=['go', 'stop'], conditions=lambda x: 'go' if x.frame_id == 'next' else 'stop'),
										transitions={'go': 'startSegment', 'stop': 'finished'},
										autonomy={'go': Autonomy.Off, 'stop': Autonomy.Off},
										remapping={'input_value': 'msg'})

			# x:105 y:157
			OperatableStateMachine.add('startDetect',
										PublishHeader(seq=2, frame_id="solar_detect"),
										transitions={'done': 'listenSolar'},
										autonomy={'done': Autonomy.Off})

			# x:447 y:155
			OperatableStateMachine.add('startSegment',
										PublishHeader(seq=2, frame_id="enable_yolov8"),
										transitions={'done': 'startDetect'},
										autonomy={'done': Autonomy.Off})

			# x:446 y:499
			OperatableStateMachine.add('stopDetect',
										PublishHeader(seq=0, frame_id="solar_detect"),
										transitions={'done': 'stopSegment'},
										autonomy={'done': Autonomy.Off})

			# x:709 y:494
			OperatableStateMachine.add('stopSegment',
										PublishHeader(seq=0, frame_id="enable_yolov8"),
										transitions={'done': 'baseBackward'},
										autonomy={'done': Autonomy.Off})

			# x:107 y:609
			OperatableStateMachine.add('wait2s',
										WaitState(wait_time=2),
										transitions={'done': 'armPickupPVM'},
										autonomy={'done': Autonomy.Off})

			# x:279 y:497
			OperatableStateMachine.add('wait5s',
										WaitState(wait_time=5),
										transitions={'done': 'stopDetect'},
										autonomy={'done': Autonomy.Off})

			# x:257 y:20
			OperatableStateMachine.add('CupOff',
										self.use_behavior(CupOffSM, 'CupOff'),
										transitions={'finished': 'FromPlace2Pick'},
										autonomy={'finished': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
