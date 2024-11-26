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
from comm_behaviors.finetunepoint_sm import FinetunePointSM
from comm_states.bracket_goal import BracketGoal
from comm_states.bracket_line_fusion import BracketLineFusion
from comm_states.manipulation_share import ManipulationShare
from comm_states.site_manipulation_ud import SiteManipulationUD
from flexbe_states.decision_state import DecisionState
from flexbe_states.log_state import LogState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jul 01 2024
@author: zcx
'''
class PlacePVMDetect_test_pointSM(Behavior):
	'''
	place PVM by detection
	'''


	def __init__(self):
		super(PlacePVMDetect_test_pointSM, self).__init__()
		self.name = 'PlacePVMDetect_test_point'

		# parameters of this behavior
		self.add_parameter('place_solar_x', 0.001)
		self.add_parameter('pvm_width', 1134)
		self.add_parameter('install_gap', 10)
		self.add_parameter('place_solar_z', 0.01)
		self.add_parameter('en_bracket', False)
		self.add_parameter('en_plane', False)
		self.add_parameter('yline_gap', 67)
		self.add_parameter('line_z', -0.1)

		# references to used behaviors
		self.add_behavior(FinetuneBLTimeoutSM, 'FinetunePitch')
		self.add_behavior(FinetuneBLTimeoutSM, 'FinetuneXY')
		self.add_behavior(FinetuneBLTimeoutSM, 'FinetuneYaw')
		self.add_behavior(FinetunePointSM, 'PointRZ')
		self.add_behavior(FinetunePointSM, 'PointXY')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1025 y:614
		_state_machine = OperatableStateMachine(outcomes=['finished'], input_keys=['solar_msg'])
		_state_machine.userdata.mani_goal = {'dx': None}
		_state_machine.userdata.goal_z_rpz = {'pos':[0,0,self.line_z], 'quat':[0,0,0,1], 'target_frame':'tool0'}
		_state_machine.userdata.solar_msg = None
		_state_machine.userdata.en_bracket = self.en_bracket
		_state_machine.userdata.en_plane = self.en_plane

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:46 y:76
			OperatableStateMachine.add('armInit',
										ManipulationShare(reference_frame="base_arm", end_effector_link="tool0"),
										transitions={'done': 'wait2s'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:500 y:565
			OperatableStateMachine.add('FinetuneXY',
										self.use_behavior(FinetuneBLTimeoutSM, 'FinetuneXY',
											parameters={'adjust_element': "x", 'yline_gap': self.yline_gap}),
										transitions={'finished': 'fusionX', 'timeout': 'FinetuneXY'},
										autonomy={'finished': Autonomy.Inherit, 'timeout': Autonomy.Inherit},
										remapping={'adjust_goal': 'xy_goal'})

			# x:500 y:332
			OperatableStateMachine.add('FinetuneYaw',
										self.use_behavior(FinetuneBLTimeoutSM, 'FinetuneYaw',
											parameters={'adjust_element': "yaw", 'yline_gap': 36}),
										transitions={'finished': 'finetuneYaw', 'timeout': 'FinetuneYaw'},
										autonomy={'finished': Autonomy.Inherit, 'timeout': Autonomy.Inherit},
										remapping={'adjust_goal': 'yaw_goal'})

			# x:262 y:428
			OperatableStateMachine.add('PointRZ',
										self.use_behavior(FinetunePointSM, 'PointRZ',
											parameters={'adjust_element': "yaw", 'yline_gap': 2}),
										transitions={'finished': 'finetuneYaw'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'adjust_goal': 'yaw_goal'})

			# x:263 y:563
			OperatableStateMachine.add('PointXY',
										self.use_behavior(FinetunePointSM, 'PointXY',
											parameters={'adjust_element': "yaw", 'yline_gap': 3}),
										transitions={'finished': 'fusionX'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'adjust_goal': 'xy_goal'})

			# x:760 y:604
			OperatableStateMachine.add('armDown',
										SiteManipulationUD(reference_frame="base_arm", end_effector_link="tool0", v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=True, cart_step_list=[3,11], step_factor=0.1, retry_num=3, itp_norm=0, if_debug=False, cart_limit={}),
										transitions={'done': 'finished', 'failed': 'armDown'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'mani_goal': 'goal_z_rpz'})

			# x:43 y:307
			OperatableStateMachine.add('armPlaceLineUD',
										SiteManipulationUD(reference_frame='base_arm', end_effector_link='tool0', v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=True, cart_step_list=[3, 11], step_factor=0.1, retry_num=3, itp_norm=0, if_debug=False, cart_limit={}),
										transitions={'done': 'wait2s', 'failed': 'armPlaceLineUD'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'mani_goal': 'line_goal'})

			# x:758 y:336
			OperatableStateMachine.add('finetunePitch',
										SiteManipulationUD(reference_frame="base_arm", end_effector_link="tool0", v_factor=1, a_factor=1, if_execute=True, wait_time=0.1, stay_level=True, cart_step_list=[3,11], step_factor=0.1, retry_num=3, itp_norm=0, if_debug=False, cart_limit={}),
										transitions={'done': 'FinetuneYaw', 'failed': 'finetunePitch'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'mani_goal': 'pitch_goal'})

			# x:508 y:756
			OperatableStateMachine.add('finetuneXY',
										SiteManipulationUD(reference_frame="base_arm", end_effector_link="tool0", v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=True, cart_step_list=[1,15], step_factor=0.01, retry_num=3, itp_norm=0, if_debug=False, cart_limit={}),
										transitions={'done': 'armDown', 'failed': 'finetuneXY'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'mani_goal': 'fusion_goal'})

			# x:510 y:430
			OperatableStateMachine.add('finetuneYaw',
										SiteManipulationUD(reference_frame="base_arm", end_effector_link="tool0", v_factor=1, a_factor=1, if_execute=True, wait_time=0.1, stay_level=True, cart_step_list=[3,11], step_factor=0.1, retry_num=3, itp_norm=0, if_debug=False, cart_limit={}),
										transitions={'done': 'PointXY', 'failed': 'finetuneYaw'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'mani_goal': 'yaw_goal'})

			# x:511 y:659
			OperatableStateMachine.add('fusionX',
										BracketLineFusion(bracket_weight=0.8, line_weight=0.2, bracket_max=0),
										transitions={'done': 'finetuneXY'},
										autonomy={'done': Autonomy.Off},
										remapping={'bracket_goal': 'mani_goal', 'line_goal': 'xy_goal', 'fusion_goal': 'fusion_goal'})

			# x:522 y:202
			OperatableStateMachine.add('log1-plane',
										LogState(text='plane timeout', severity=Logger.REPORT_HINT),
										transitions={'done': 'FinetuneYaw'},
										autonomy={'done': Autonomy.Off})

			# x:470 y:110
			OperatableStateMachine.add('log2-plane',
										LogState(text='start plane', severity=Logger.REPORT_HINT),
										transitions={'done': 'FinetunePitch'},
										autonomy={'done': Autonomy.Off})

			# x:277 y:109
			OperatableStateMachine.add('planeDec',
										DecisionState(outcomes=['true', 'false'], conditions=lambda x: 'true' if x else 'false'),
										transitions={'true': 'log2-plane', 'false': 'log2-plane'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'input_value': 'en_plane'})

			# x:51 y:214
			OperatableStateMachine.add('solarLinkInfo',
										BracketGoal(pos=[self.place_solar_x, (self.pvm_width+ self.install_gap)*0.001+0.1, self.place_solar_z+0.05], quat=[0,0.0087265,0,0.9999619], stable_time=3, dx_max=0.1, if_bkt=False, limit_dict={}, pos_targets=[], itp_norm=0.0),
										transitions={'done': 'armPlaceLineUD'},
										autonomy={'done': Autonomy.Off},
										remapping={'pose_msg': 'solar_msg', 'mani_goal': 'line_goal'})

			# x:276 y:308
			OperatableStateMachine.add('wait2s',
										WaitState(wait_time=2),
										transitions={'done': 'PointRZ'},
										autonomy={'done': Autonomy.Off})

			# x:746 y:109
			OperatableStateMachine.add('FinetunePitch',
										self.use_behavior(FinetuneBLTimeoutSM, 'FinetunePitch',
											parameters={'adjust_element': "rpz", 'yline_gap': 36}),
										transitions={'finished': 'finetunePitch', 'timeout': 'log1-plane'},
										autonomy={'finished': Autonomy.Inherit, 'timeout': Autonomy.Inherit},
										remapping={'adjust_goal': 'pitch_goal'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
