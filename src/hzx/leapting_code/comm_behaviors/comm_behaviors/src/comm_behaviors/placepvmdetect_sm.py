#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_behaviors.finetunebltimeout_sm import FinetuneBLTimeoutSM
from comm_states.manipulation_share import ManipulationShare
from comm_states.publisherheader import PublishHeader
from comm_states.site_manipulation_ud import SiteManipulationUD
from flexbe_states.decision_state import DecisionState
from flexbe_states.log_state import LogState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jul 01 2024
@author: zcx
'''
class PlacePVMDetectSM(Behavior):
	'''
	place PVM by detection
	'''


	def __init__(self):
		super(PlacePVMDetectSM, self).__init__()
		self.name = 'PlacePVMDetect'

		# parameters of this behavior
		self.add_parameter('place_solar_x', 0.001)
		self.add_parameter('pvm_width', 1134)
		self.add_parameter('install_gap', 10)
		self.add_parameter('place_solar_z', 0.01)
		self.add_parameter('en_bracket', False)
		self.add_parameter('en_plane', True)
		self.add_parameter('yline_gap', 67)
		self.add_parameter('line_z', -0.09)

		# references to used behaviors
		self.add_behavior(FinetuneBLTimeoutSM, 'FinetunePitch')
		self.add_behavior(FinetuneBLTimeoutSM, 'FinetuneXY')
		self.add_behavior(FinetuneBLTimeoutSM, 'FinetuneYaw')
		self.add_behavior(FinetuneBLTimeoutSM, 'FinetuneZ')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1025 y:614
		_state_machine = OperatableStateMachine(outcomes=['finished'])
		_state_machine.userdata.mani_goal = {'dx': None}
		_state_machine.userdata.goal_z_rpz = {'pos':[0,0,self.line_z], 'quat':[0,0,0,1], 'target_frame':'tool0'}
		_state_machine.userdata.en_bracket = self.en_bracket
		_state_machine.userdata.en_plane = self.en_plane

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:46 y:76
			OperatableStateMachine.add('armInit',
										ManipulationShare(reference_frame="base_arm", end_effector_link="tool0"),
										transitions={'done': 'planeDec'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:500 y:565
			OperatableStateMachine.add('FinetuneXY',
										self.use_behavior(FinetuneBLTimeoutSM, 'FinetuneXY',
											parameters={'adjust_element': "x", 'yline_gap': self.yline_gap, 'adjust_topic': "res_line", 'stop_element': "line"}),
										transitions={'finished': 'finetuneXY', 'timeout': 'FinetuneXY'},
										autonomy={'finished': Autonomy.Inherit, 'timeout': Autonomy.Inherit},
										remapping={'adjust_goal': 'xy_goal'})

			# x:500 y:332
			OperatableStateMachine.add('FinetuneYaw',
										self.use_behavior(FinetuneBLTimeoutSM, 'FinetuneYaw',
											parameters={'adjust_element': "yaw", 'yline_gap': 36, 'adjust_topic': "res_line", 'stop_element': "line"}),
										transitions={'finished': 'finetuneYaw', 'timeout': 'FinetuneYaw'},
										autonomy={'finished': Autonomy.Inherit, 'timeout': Autonomy.Inherit},
										remapping={'adjust_goal': 'yaw_goal'})

			# x:846 y:581
			OperatableStateMachine.add('FinetuneZ',
										self.use_behavior(FinetuneBLTimeoutSM, 'FinetuneZ',
											parameters={'adjust_element': "z", 'yline_gap': 36, 'adjust_topic': "res_plane", 'stop_element': "adjust"}),
										transitions={'finished': 'stopEdge3', 'timeout': 'startEdge2'},
										autonomy={'finished': Autonomy.Inherit, 'timeout': Autonomy.Inherit},
										remapping={'adjust_goal': 'goal_z_rpz'})

			# x:772 y:766
			OperatableStateMachine.add('armDown',
										SiteManipulationUD(reference_frame="base_arm", end_effector_link="tool0", v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=True, cart_step_list=[3,11], step_factor=0.1, retry_num=3, itp_norm=0, if_debug=False, cart_limit={}),
										transitions={'done': 'finished', 'failed': 'armDown'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'mani_goal': 'goal_z_rpz'})

			# x:758 y:336
			OperatableStateMachine.add('finetunePitch',
										SiteManipulationUD(reference_frame="base_arm", end_effector_link="tool0", v_factor=1, a_factor=1, if_execute=True, wait_time=0.1, stay_level=True, cart_step_list=[3,11], step_factor=0.1, retry_num=3, itp_norm=0, if_debug=False, cart_limit={}),
										transitions={'done': 'FinetuneYaw', 'failed': 'finetunePitch'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'mani_goal': 'pitch_goal'})

			# x:510 y:756
			OperatableStateMachine.add('finetuneXY',
										SiteManipulationUD(reference_frame="base_arm", end_effector_link="tool0", v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=True, cart_step_list=[1,15], step_factor=0.01, retry_num=3, itp_norm=0, if_debug=False, cart_limit={}),
										transitions={'done': 'startEdge2', 'failed': 'finetuneXY'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'mani_goal': 'xy_goal'})

			# x:510 y:430
			OperatableStateMachine.add('finetuneYaw',
										SiteManipulationUD(reference_frame="base_arm", end_effector_link="tool0", v_factor=1, a_factor=1, if_execute=True, wait_time=0.1, stay_level=True, cart_step_list=[3,11], step_factor=0.1, retry_num=3, itp_norm=0, if_debug=False, cart_limit={}),
										transitions={'done': 'FinetuneXY', 'failed': 'finetuneYaw'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'mani_goal': 'yaw_goal'})

			# x:522 y:202
			OperatableStateMachine.add('log1-plane',
										LogState(text='plane timeout', severity=Logger.REPORT_HINT),
										transitions={'done': 'startEdge'},
										autonomy={'done': Autonomy.Off})

			# x:470 y:110
			OperatableStateMachine.add('log2-plane',
										LogState(text='start plane', severity=Logger.REPORT_HINT),
										transitions={'done': 'startEdge'},
										autonomy={'done': Autonomy.Off})

			# x:277 y:109
			OperatableStateMachine.add('planeDec',
										DecisionState(outcomes=['true', 'false'], conditions=lambda x: 'true' if x else 'false'),
										transitions={'true': 'log2-plane', 'false': 'log2-plane'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'input_value': 'en_plane'})

			# x:600 y:20
			OperatableStateMachine.add('startEdge',
										PublishHeader(seq=2, frame_id="adjust_yaw"),
										transitions={'done': 'FinetunePitch'},
										autonomy={'done': Autonomy.Off})

			# x:780 y:483
			OperatableStateMachine.add('startEdge2',
										PublishHeader(seq=2, frame_id="adjust_yaw"),
										transitions={'done': 'FinetuneZ'},
										autonomy={'done': Autonomy.Off})

			# x:771 y:213
			OperatableStateMachine.add('stopEdge',
										PublishHeader(seq=0, frame_id="enable_line"),
										transitions={'done': 'finetunePitch'},
										autonomy={'done': Autonomy.Off})

			# x:637 y:148
			OperatableStateMachine.add('stopEdge2',
										PublishHeader(seq=0, frame_id="enable_line"),
										transitions={'done': 'log1-plane'},
										autonomy={'done': Autonomy.Off})

			# x:782 y:671
			OperatableStateMachine.add('stopEdge3',
										PublishHeader(seq=0, frame_id="enable_line"),
										transitions={'done': 'armDown'},
										autonomy={'done': Autonomy.Off})

			# x:746 y:109
			OperatableStateMachine.add('FinetunePitch',
										self.use_behavior(FinetuneBLTimeoutSM, 'FinetunePitch',
											parameters={'adjust_element': "rp", 'yline_gap': 36, 'adjust_topic': "res_plane", 'stop_element': "adjust"}),
										transitions={'finished': 'stopEdge', 'timeout': 'stopEdge2'},
										autonomy={'finished': Autonomy.Inherit, 'timeout': Autonomy.Inherit},
										remapping={'adjust_goal': 'pitch_goal'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
