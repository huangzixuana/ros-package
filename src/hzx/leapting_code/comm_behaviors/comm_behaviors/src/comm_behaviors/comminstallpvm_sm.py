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
from comm_behaviors.commpickuppvm_sm import CommPickupPVMSM
from comm_behaviors.commplace2pvm_sm import CommPlace2PVMSM
from comm_behaviors.cupoff_sm import CupOffSM
from comm_behaviors.fromplace2pick_sm import fromPlace2PickSM
from comm_behaviors.fromplacemid2pick_sm import fromPlaceMid2PickSM
from comm_behaviors.navlastpvm_sm import NavLastPVMSM
from comm_behaviors.updown_sm import UpDownSM
from comm_behaviors.withdrawpvm_sm import WithdrawPVMSM
from comm_states.manipulation_share import ManipulationShare
from comm_states.publisherheader import PublishHeader
from flexbe_states.calculation_state import CalculationState
from flexbe_states.decision_state import DecisionState
from flexbe_states.log_state import LogState
from flexbe_states.subscriber_state import SubscriberState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jun 07 2023
@author: ZengLei
'''
class CommInstallPVMSM(Behavior):
	'''
	CommInstallPVM
	'''


	def __init__(self):
		super(CommInstallPVMSM, self).__init__()
		self.name = 'CommInstallPVM'

		# parameters of this behavior
		self.add_parameter('auto', True)
		self.add_parameter('pvm_sum', 1)
		self.add_parameter('pick_solar_x', -0.1)
		self.add_parameter('pick_solar_y', 0.08)
		self.add_parameter('pick_solar_z', 0.025)
		self.add_parameter('pick_ideal_x', 2.1)
		self.add_parameter('pick_ideal_y', -0.006)
		self.add_parameter('pick_ideal_z', 0.0)
		self.add_parameter('pick_ideal_er', 0.006)
		self.add_parameter('pick_ideal_ep', -0.011)
		self.add_parameter('pick_ideal_ey', 1.559)
		self.add_parameter('pick_tol_x', -1)
		self.add_parameter('pick_tol_y', 0.4)
		self.add_parameter('pick_tol_z', -1.0)
		self.add_parameter('pick_tol_er', 0.2)
		self.add_parameter('pick_tol_ep', 0.2)
		self.add_parameter('pick_tol_ey', 0.2)
		self.add_parameter('pvm_width', 1134)
		self.add_parameter('install_gap', 10)
		self.add_parameter('place_solar_x', -0.03)
		self.add_parameter('place_solar_z', 0.08)
		self.add_parameter('place_ideal_x', 0.6)
		self.add_parameter('place_ideal_y', 2.4)
		self.add_parameter('place_ideal_z', 0)
		self.add_parameter('place_ideal_er', -0.05)
		self.add_parameter('place_ideal_ep', -0.04)
		self.add_parameter('place_ideal_ey', 1.63)
		self.add_parameter('place_tol_x', -1)
		self.add_parameter('place_tol_y', -1)
		self.add_parameter('place_tol_z', -1)
		self.add_parameter('place_tol_er', 0.5)
		self.add_parameter('place_tol_ep', 0.5)
		self.add_parameter('place_tol_ey', 0.5)
		self.add_parameter('detect_option', 'line')
		self.add_parameter('line_gap', 51)

		# references to used behaviors
		self.add_behavior(UpDownSM, 'ArmUp10cm')
		self.add_behavior(change_bzSM, 'ChangeBZ')
		self.add_behavior(CommPickupPVMSM, 'CommPickupPVM')
		self.add_behavior(CommPlace2PVMSM, 'CommPlace2PVM')
		self.add_behavior(CupOffSM, 'CupOff')
		self.add_behavior(fromPlace2PickSM, 'FromPlace2Pick')
		self.add_behavior(NavLastPVMSM, 'NavLastPVM')
		self.add_behavior(CommPickupPVMSM, 'PickupPVM')
		self.add_behavior(CommPickupPVMSM, 'StaticPickupPVM')
		self.add_behavior(WithdrawPVMSM, 'WithdrawPVM')
		self.add_behavior(fromPlaceMid2PickSM, 'fromPlaceMid2Pick')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1475 y:282, x:849 y:528
		_state_machine = OperatableStateMachine(outcomes=['finished', 'canceled'], input_keys=['robot_pose', 'tool_pose'])
		_state_machine.userdata.nav_goal = {'frame_id':'base_link', 'position':[0,0,0],'orientation':[0,0,0,1]}
		_state_machine.userdata.auto = self.auto
		_state_machine.userdata.pvm_num = 0
		_state_machine.userdata.detect_option = self.detect_option
		_state_machine.userdata.pose_msg_in = None
		_state_machine.userdata.pose_msg_pick = None
		_state_machine.userdata.robot_pose = None
		_state_machine.userdata.tool_pose = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:70 y:87
			OperatableStateMachine.add('armInit',
										ManipulationShare(reference_frame="base_arm", end_effector_link="tool0"),
										transitions={'done': 'ChangeBZ'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:219 y:159
			OperatableStateMachine.add('ChangeBZ',
										self.use_behavior(change_bzSM, 'ChangeBZ'),
										transitions={'finished': 'CommPickupPVM'},
										autonomy={'finished': Autonomy.Inherit})

			# x:217 y:272
			OperatableStateMachine.add('CommPickupPVM',
										self.use_behavior(CommPickupPVMSM, 'CommPickupPVM',
											parameters={'mode': "StaticPickupPVM", 'pick_solar_x': self.pick_solar_x, 'pick_solar_y': self.pick_solar_y, 'pick_solar_z': self.pick_solar_z, 'pick_ideal_x': self.pick_ideal_x, 'pick_ideal_y': self.pick_ideal_y, 'pick_ideal_z': self.pick_ideal_z, 'pick_ideal_er': self.pick_ideal_er, 'pick_ideal_ep': self.pick_ideal_ep, 'pick_ideal_ey': self.pick_ideal_ey, 'pick_tol_x': self.pick_tol_x, 'pick_tol_y': self.pick_tol_y, 'pick_tol_z': self.pick_tol_z, 'pick_tol_er': self.pick_tol_er, 'pick_tol_ep': self.pick_tol_ep, 'pick_tol_ey': self.pick_tol_ey, 'detect_option': self.detect_option}),
										transitions={'finished': 'auto1'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'nav_goal': 'nav_goal', 'pose_msg_in': 'pose_msg_in', 'robot_pose': 'robot_pose', 'tool_pose': 'tool_pose', 'pose_msg_pick': 'pose_msg_pick'})

			# x:592 y:329
			OperatableStateMachine.add('CommPlace2PVM',
										self.use_behavior(CommPlace2PVMSM, 'CommPlace2PVM',
											parameters={'mode': "PlacePVM", 'pvm_width': self.pvm_width, 'install_gap': self.install_gap, 'place_solar_x': self.place_solar_x, 'place_solar_z': self.place_solar_z, 'place_ideal_x': self.place_ideal_x, 'place_ideal_y': self.place_ideal_y, 'place_ideal_z': self.place_ideal_z, 'place_ideal_er': self.place_ideal_er, 'place_ideal_ep': self.place_ideal_ep, 'place_ideal_ey': self.place_ideal_ey, 'place_tol_x': self.place_tol_x, 'place_tol_y': self.place_tol_y, 'place_tol_z': self.place_tol_z, 'place_tol_er': self.place_tol_er, 'place_tol_ep': self.place_tol_ep, 'place_tol_ey': self.place_tol_ey, 'detect_option': self.detect_option, 'line_gap': self.line_gap, 'sec_offset_x': 0.0, 'sec_offset_y': 0.0, 'sec_offset_z': 0.0}),
										transitions={'finished': 'auto2'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'pose_msg_in': 'pose_msg_pick', 'pvm_num': 'pvm_num', 'robot_pose': 'robot_pose', 'tool_pose': 'tool_pose', 'nav_goal': 'nav_goal'})

			# x:1228 y:54
			OperatableStateMachine.add('CupOff',
										self.use_behavior(CupOffSM, 'CupOff'),
										transitions={'finished': 'ArmUp10cm'},
										autonomy={'finished': Autonomy.Inherit})

			# x:1574 y:271
			OperatableStateMachine.add('FromPlace2Pick',
										self.use_behavior(fromPlace2PickSM, 'FromPlace2Pick'),
										transitions={'finished': 'finished'},
										autonomy={'finished': Autonomy.Inherit})

			# x:614 y:415
			OperatableStateMachine.add('LogOcc',
										LogState(text="place log", severity=2),
										transitions={'done': 'CommPlace2PVM'},
										autonomy={'done': Autonomy.Off})

			# x:1233 y:123
			OperatableStateMachine.add('NavLastPVM',
										self.use_behavior(NavLastPVMSM, 'NavLastPVM'),
										transitions={'finished': 'finished'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'nav_goal': 'nav_goal'})

			# x:1236 y:200
			OperatableStateMachine.add('PickupPVM',
										self.use_behavior(CommPickupPVMSM, 'PickupPVM',
											parameters={'mode': "PickupPVM", 'pick_solar_x': self.pick_solar_x, 'pick_solar_y': self.pick_solar_y, 'pick_solar_z': self.pick_solar_z, 'pick_ideal_x': self.pick_ideal_x, 'pick_ideal_y': self.pick_ideal_y, 'pick_ideal_z': self.pick_ideal_z, 'pick_ideal_er': self.pick_ideal_er, 'pick_ideal_ep': self.pick_ideal_ep, 'pick_ideal_ey': self.pick_ideal_ey, 'pick_tol_x': self.pick_tol_x, 'pick_tol_y': self.pick_tol_y, 'pick_tol_z': self.pick_tol_z, 'pick_tol_er': self.pick_tol_er, 'pick_tol_ep': self.pick_tol_ep, 'pick_tol_ey': self.pick_tol_ey, 'detect_option': self.detect_option}),
										transitions={'finished': 'auto1'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'nav_goal': 'nav_goal', 'pose_msg_in': 'pose_msg_in', 'robot_pose': 'robot_pose', 'tool_pose': 'tool_pose', 'pose_msg_pick': 'pose_msg_pick'})

			# x:597 y:536
			OperatableStateMachine.add('StaticPickupPVM',
										self.use_behavior(CommPickupPVMSM, 'StaticPickupPVM',
											parameters={'mode': "StaticPickupPVM", 'pick_solar_x': self.pick_solar_x, 'pick_solar_y': self.pick_solar_y, 'pick_solar_z': self.pick_solar_z, 'pick_ideal_x': self.pick_ideal_x, 'pick_ideal_y': self.pick_ideal_y, 'pick_ideal_z': self.pick_ideal_z, 'pick_ideal_er': self.pick_ideal_er, 'pick_ideal_ep': self.pick_ideal_ep, 'pick_ideal_ey': self.pick_ideal_ey, 'pick_tol_x': self.pick_tol_x, 'pick_tol_y': self.pick_tol_y, 'pick_tol_z': self.pick_tol_z, 'pick_tol_er': self.pick_tol_er, 'pick_tol_ep': self.pick_tol_ep, 'pick_tol_ey': self.pick_tol_ey, 'detect_option': self.detect_option}),
										transitions={'finished': 'LogOcc'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'nav_goal': 'nav_goal', 'pose_msg_in': 'pose_msg_in', 'robot_pose': 'robot_pose', 'tool_pose': 'tool_pose', 'pose_msg_pick': 'pose_msg_pick'})

			# x:804 y:742
			OperatableStateMachine.add('UI:next?1',
										SubscriberState(topic='trig', blocking=True, clear=True),
										transitions={'received': 'decision1', 'unavailable': 'UI:next?1'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'msg1'})

			# x:1237 y:414
			OperatableStateMachine.add('UI:next?withdraw?2',
										SubscriberState(topic='trig', blocking=True, clear=True),
										transitions={'received': 'decision2', 'unavailable': 'UI:next?withdraw?2'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'msg2'})

			# x:755 y:92
			OperatableStateMachine.add('UI:next?withdraw?3',
										SubscriberState(topic='trig', blocking=True, clear=True),
										transitions={'received': 'decision3', 'unavailable': 'UI:next?withdraw?3'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'msg3'})

			# x:938 y:544
			OperatableStateMachine.add('WithdrawPVM',
										self.use_behavior(WithdrawPVMSM, 'WithdrawPVM',
											parameters={'install_mode': True}),
										transitions={'finished': 'informWeb'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'nav_goal': 'nav_goal'})

			# x:1057 y:277
			OperatableStateMachine.add('auto1',
										DecisionState(outcomes=['True', 'False'], conditions=lambda x: 'True' if x else 'False'),
										transitions={'True': 'LogOcc', 'False': 'informWeb2'},
										autonomy={'True': Autonomy.Off, 'False': Autonomy.Off},
										remapping={'input_value': 'auto'})

			# x:612 y:201
			OperatableStateMachine.add('auto2',
										DecisionState(outcomes=['True', 'False'], conditions=lambda x: 'True' if x else 'False'),
										transitions={'True': 'counter', 'False': 'informWeb3'},
										autonomy={'True': Autonomy.Off, 'False': Autonomy.Off},
										remapping={'input_value': 'auto'})

			# x:898 y:203
			OperatableStateMachine.add('compare',
										DecisionState(outcomes=['next', 'done'], conditions=lambda x: 'next' if x<self.pvm_sum else 'done'),
										transitions={'next': 'log', 'done': 'CupOff'},
										autonomy={'next': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'input_value': 'pvm_num'})

			# x:772 y:203
			OperatableStateMachine.add('counter',
										CalculationState(calculation=lambda x: x+1),
										transitions={'done': 'compare'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'pvm_num', 'output_value': 'pvm_num'})

			# x:615 y:646
			OperatableStateMachine.add('decision1',
										DecisionState(outcomes=['next','invalid'], conditions=lambda x: x.frame_id if (x.frame_id in ['next']) else 'invalid'),
										transitions={'next': 'StaticPickupPVM', 'invalid': 'informWeb'},
										autonomy={'next': Autonomy.Off, 'invalid': Autonomy.Off},
										remapping={'input_value': 'msg1'})

			# x:849 y:412
			OperatableStateMachine.add('decision2',
										DecisionState(outcomes=['next', 'withdraw','invalid'], conditions=lambda x: x.frame_id if (x.frame_id in ['next', 'withdraw']) else 'invalid'),
										transitions={'next': 'LogOcc', 'withdraw': 'canceled', 'invalid': 'informWeb2'},
										autonomy={'next': Autonomy.Off, 'withdraw': Autonomy.Off, 'invalid': Autonomy.Off},
										remapping={'input_value': 'msg2'})

			# x:964 y:45
			OperatableStateMachine.add('decision3',
										DecisionState(outcomes=['next', 'withdraw','invalid'], conditions=lambda x: x.frame_id if (x.frame_id in ['next', 'withdraw']) else 'invalid'),
										transitions={'next': 'PickupPVM', 'withdraw': 'WithdrawPVM', 'invalid': 'informWeb3'},
										autonomy={'next': Autonomy.Off, 'withdraw': Autonomy.Off, 'invalid': Autonomy.Off},
										remapping={'input_value': 'msg3'})

			# x:1561 y:163
			OperatableStateMachine.add('fromPlaceMid2Pick',
										self.use_behavior(fromPlaceMid2PickSM, 'fromPlaceMid2Pick'),
										transitions={'finished': 'finished'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'move_group': 'move_group'})

			# x:956 y:647
			OperatableStateMachine.add('informWeb',
										PublishHeader(seq=1, frame_id="UI_pick:next?"),
										transitions={'done': 'UI:next?1'},
										autonomy={'done': Autonomy.Off})

			# x:1259 y:279
			OperatableStateMachine.add('informWeb2',
										PublishHeader(seq=1, frame_id="UI_place:next?withdraw?"),
										transitions={'done': 'UI:next?withdraw?2'},
										autonomy={'done': Autonomy.Off})

			# x:613 y:45
			OperatableStateMachine.add('informWeb3',
										PublishHeader(seq=1, frame_id="UI_dump:next?withdraw?"),
										transitions={'done': 'UI:next?withdraw?3'},
										autonomy={'done': Autonomy.Off})

			# x:1059 y:201
			OperatableStateMachine.add('log',
										LogState(text='=== pvm: +1===', severity=Logger.REPORT_HINT),
										transitions={'done': 'PickupPVM'},
										autonomy={'done': Autonomy.Off})

			# x:1570 y:54
			OperatableStateMachine.add('ArmUp10cm',
										self.use_behavior(UpDownSM, 'ArmUp10cm',
											parameters={'height': 0.1}),
										transitions={'done': 'fromPlaceMid2Pick'},
										autonomy={'done': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
