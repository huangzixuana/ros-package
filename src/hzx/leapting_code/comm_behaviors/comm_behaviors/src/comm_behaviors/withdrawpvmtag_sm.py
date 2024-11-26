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
from comm_behaviors.startpvmdetect_sm import StartPVMDetectSM
from comm_behaviors.stoppvmdetect_sm import StopPVMDetectSM
from comm_states.listen_solar import ListenSolar
from comm_states.manipulation_share import ManipulationShare
from comm_states.publisherheader import PublishHeader
from comm_states.site_manipulation import SiteManipulation
from comm_states.site_navigation import SiteNavigation
from comm_states.tag_goal import TagGoal
from flexbe_states.decision_state import DecisionState
from flexbe_states.subscriber_state import SubscriberState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Sep 20 2024
@author: zl
'''
class WithdrawPVMTagSM(Behavior):
	'''
	test
	'''


	def __init__(self):
		super(WithdrawPVMTagSM, self).__init__()
		self.name = 'WithdrawPVMTag'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(CupOffSM, 'CupOff')
		self.add_behavior(StartPVMDetectSM, 'StartPVMDetect')
		self.add_behavior(StopPVMDetectSM, 'StopPVMDetect')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:809 y:625, x:760 y:767
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['nav_goal'])
		_state_machine.userdata.nav_goal = {}

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:81 y:47
			OperatableStateMachine.add('armInit',
										ManipulationShare(reference_frame="base_arm", end_effector_link="tool0"),
										transitions={'done': 'armTagPose'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:544 y:547
			OperatableStateMachine.add('Dec',
										DecisionState(outcomes=['next','invalid'], conditions=lambda x: x.frame_id if (x.frame_id in ['next']) else 'invalid'),
										transitions={'next': 'down', 'invalid': 'infoWeb'},
										autonomy={'next': Autonomy.Off, 'invalid': Autonomy.Off},
										remapping={'input_value': 'message'})

			# x:573 y:47
			OperatableStateMachine.add('StartPVMDetect',
										self.use_behavior(StartPVMDetectSM, 'StartPVMDetect'),
										transitions={'finished': 'wait3s'},
										autonomy={'finished': Autonomy.Inherit})

			# x:90 y:279
			OperatableStateMachine.add('StopPVMDetect',
										self.use_behavior(StopPVMDetectSM, 'StopPVMDetect'),
										transitions={'finished': 'top2'},
										autonomy={'finished': Autonomy.Inherit})

			# x:422 y:632
			OperatableStateMachine.add('UI:next?1',
										SubscriberState(topic='trig', blocking=True, clear=True),
										transitions={'received': 'Dec', 'unavailable': 'UI:next?1'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:309 y:414
			OperatableStateMachine.add('armGo3Pose',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="none", axis_value=["none",0], pos_targets=['armTopPose', 'temp_itp'], trajectory_name="none", reference_frame="base_arm", end_effector_link="tool0", wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=True, cart_step_list=[1,250], step_factor=0.01, itp_norm=0, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id="none", plan_time=2),
										transitions={'done': 'down', 'failed': 'armGo3Pose'},
										autonomy={'done': Autonomy.Full, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:315 y:48
			OperatableStateMachine.add('armTagPose',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armTagPose", axis_value=["none",0], pos_targets=[], trajectory_name="none", reference_frame="base_arm", end_effector_link="tool0", wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=False, cart_step_list=[3,11], step_factor=0.1, itp_norm=0.15, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id="none", plan_time=2),
										transitions={'done': 'StartPVMDetect', 'failed': 'armTagPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:770 y:409
			OperatableStateMachine.add('armTopPose',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armTopPose", axis_value=["none",0], pos_targets=[], trajectory_name="none", reference_frame="base_arm", end_effector_link="tool0", wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=False, cart_step_list=[3,11], step_factor=0.1, itp_norm=0.15, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id="none", plan_time=2),
										transitions={'done': 'navForward', 'failed': 'armTopPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:768 y:286
			OperatableStateMachine.add('armUp',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="none", axis_value=["none",0], pos_targets=['temp_itp', 'armTopPose'], trajectory_name="none", reference_frame="base_arm", end_effector_link="tool0", wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=True, cart_step_list=[1,19], step_factor=0.1, itp_norm=0.0, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id="none", plan_time=2),
										transitions={'done': 'armTopPose', 'failed': 'armUp'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:322 y:151
			OperatableStateMachine.add('checkSolar',
										ListenSolar(timeout=60, fresh_time=3.0, ideal=[0,0,0,0,0,0], tolerance=[-1,-1,-1,-1,-1,-1], pose_topic="filter_solar_pose"),
										transitions={'done': 'tag2pose', 'timeout': 'checkSolar'},
										autonomy={'done': Autonomy.Off, 'timeout': Autonomy.Off},
										remapping={'pose_msg': 'pose_msg'})

			# x:537 y:409
			OperatableStateMachine.add('down',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="none", axis_value=["none",0], pos_targets=['temp_solar'], trajectory_name="none", reference_frame="base_arm", end_effector_link="tool0", wait_time=0.5, v_factor=1, a_factor=1, t_factor=1.0, stay_level=True, cart_step_list=[1,100], step_factor=0.01, itp_norm=0, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id="none", plan_time=2),
										transitions={'done': 'CupOff', 'failed': 'down'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:314 y:547
			OperatableStateMachine.add('infoWeb',
										PublishHeader(seq=1, frame_id="UI_dump:next?"),
										transitions={'done': 'UI:next?1'},
										autonomy={'done': Autonomy.Off})

			# x:773 y:518
			OperatableStateMachine.add('navForward',
										SiteNavigation(site_name='', position=[0, 0, 0], orientation=[0, 0, 0, 1], frame_id='map', base_link2map=True),
										transitions={'arrived': 'finished', 'canceled': 'navForward', 'failed': 'navForward'},
										autonomy={'arrived': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'nav_goal': 'nav_goal'})

			# x:103 y:152
			OperatableStateMachine.add('tag2pose',
										TagGoal(pos=[0,0,0.3], quat=[ 0, 0, 0, 1 ], limit_dict={'z_max': 1.45}, pos_targets=['armTopPose'], itp_norm=2.0),
										transitions={'done': 'StopPVMDetect'},
										autonomy={'done': Autonomy.Off},
										remapping={'pose_msg': 'pose_msg', 'mani_goal': 'mani_goal'})

			# x:97 y:412
			OperatableStateMachine.add('top2',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armTopPose2", axis_value=["none",0], pos_targets=[], trajectory_name="none", reference_frame="base_arm", end_effector_link="tool0", wait_time=0.5, v_factor=1, a_factor=1, t_factor=1.0, stay_level=False, cart_step_list=[3,11], step_factor=0.1, itp_norm=0.15, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id="none", plan_time=2),
										transitions={'done': 'armGo3Pose', 'failed': 'top2'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:590 y:156
			OperatableStateMachine.add('wait3s',
										WaitState(wait_time=3),
										transitions={'done': 'checkSolar'},
										autonomy={'done': Autonomy.Off})

			# x:583 y:287
			OperatableStateMachine.add('CupOff',
										self.use_behavior(CupOffSM, 'CupOff'),
										transitions={'finished': 'armUp'},
										autonomy={'finished': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
