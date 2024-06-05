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
from comm_behaviors.cupon_sm import CupOnSM
from comm_behaviors.fromplace2pick_sm import fromPlace2PickSM
from comm_behaviors.test1025_sm import test1025SM
from comm_behaviors.trailerpvmdetect_sm import TrailerPVMDetectSM
from comm_states.listen_solar import ListenSolar
from comm_states.manipulation_share import ManipulationShare
from comm_states.publisherheader import PublishHeader
from comm_states.site_manipulation import SiteManipulation
from comm_states.site_navigation import SiteNavigation
from flexbe_states.decision_state import DecisionState
from flexbe_states.log_state import LogState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jun 02 2023
@author: ZengLei
'''
class CommPickupPVMSM(Behavior):
	'''
	PickupPVM
	'''


	def __init__(self):
		super(CommPickupPVMSM, self).__init__()
		self.name = 'CommPickupPVM'

		# parameters of this behavior
		self.add_parameter('mode', 'SimPickupPVM')
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
		self.add_behavior(CupOffSM, 'CupOff')
		self.add_behavior(CupOnSM, 'CupOn')
		self.add_behavior(TrailerPVMDetectSM, 'TrailerPVMDetect')
		self.add_behavior(fromPlace2PickSM, 'fromPlace2Pick')
		self.add_behavior(test1025SM, 'test1025')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1040 y:40
		_state_machine = OperatableStateMachine(outcomes=['finished'], input_keys=['nav_goal'])
		_state_machine.userdata.nav_goal = {'frame_id':'base_link', 'position':[0,0,0],'orientation':[0,0,0,1]}
		_state_machine.userdata.mode = self.mode

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:60 y:36
			OperatableStateMachine.add('armInit',
										ManipulationShare(reference_frame='base_arm', end_effector_link='tool0'),
										transitions={'done': 'modeDecision1'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:583 y:66
			OperatableStateMachine.add('CupOn',
										self.use_behavior(CupOnSM, 'CupOn'),
										transitions={'finished': 'modeDecision3'},
										autonomy={'finished': Autonomy.Inherit})

			# x:253 y:207
			OperatableStateMachine.add('TrailerPVMDetect',
										self.use_behavior(TrailerPVMDetectSM, 'TrailerPVMDetect',
											parameters={'pick_solar_x': self.pick_solar_x, 'pick_solar_y': self.pick_solar_y, 'pick_solar_z': self.pick_solar_z, 'pick_ideal_x': self.pick_ideal_x, 'pick_ideal_y': self.pick_ideal_y, 'pick_ideal_z': self.pick_ideal_z, 'pick_ideal_er': self.pick_ideal_er, 'pick_ideal_ep': self.pick_ideal_ep, 'pick_ideal_ey': self.pick_ideal_ey, 'pick_tol_x': self.pick_tol_x, 'pick_tol_y': self.pick_tol_y, 'pick_tol_z': self.pick_tol_z, 'pick_tol_er': self.pick_tol_er, 'pick_tol_ep': self.pick_tol_ep, 'pick_tol_ey': self.pick_tol_ey}),
										transitions={'finished': 'modeDecision2'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'nav_goal': 'nav_goal'})

			# x:489 y:540
			OperatableStateMachine.add('armUPend',
										SiteManipulation(pos=[0, 0, 0.3], quat=[0, 0, 0, 1], target_frame='tool0', target_name='none', axis_value=['none', 0], pos_targets=[], reference_frame='base_arm', end_effector_link='tool0', v_factor=1, a_factor=1, if_execute=True, wait_time=0.5, stay_level=True, cart_step_list=[1, 3], retry_num=5, itp_norm=0, if_debug=True, cart_limit={}),
										transitions={'done': 'test1025', 'failed': 'armUPend'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:503 y:267
			OperatableStateMachine.add('baseForward',
										SiteNavigation(site_name="", position=[0,0,0], orientation=[0,0,0,1], frame_id="base_link", base_link2map=False),
										transitions={'arrived': 'waitForward', 'canceled': 'baseForward', 'failed': 'baseForward'},
										autonomy={'arrived': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'nav_goal': 'nav_goal'})

			# x:916 y:194
			OperatableStateMachine.add('checkPlaceSOlar',
										ListenSolar(timeout=60, fresh_time=3.0, ideal=[0.6, 2.4, 0, -0.05, -0.04, 1.63], tolerance=[0.5, 0.3, -1, 0.5, 0.5, 0.5]),
										transitions={'done': 'finished', 'timeout': 'checkPlaceSOlar'},
										autonomy={'done': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:258 y:396
			OperatableStateMachine.add('fromPlace2Pick',
										self.use_behavior(fromPlace2PickSM, 'fromPlace2Pick'),
										transitions={'finished': 'TrailerPVMDetect'},
										autonomy={'finished': Autonomy.Inherit})

			# x:497 y:450
			OperatableStateMachine.add('log',
										LogState(text="leave PVMs", severity=Logger.REPORT_HINT),
										transitions={'done': 'armUPend'},
										autonomy={'done': Autonomy.Off})

			# x:29 y:130
			OperatableStateMachine.add('modeDecision1',
										DecisionState(outcomes=['PickupPVM' ,'SimPickupPVM', 'StaticPickupPVM','invalid'], conditions=lambda x: x if (x in ['PickupPVM' ,'SimPickupPVM', 'StaticPickupPVM']) else 'invalid'),
										transitions={'PickupPVM': 'CupOff', 'SimPickupPVM': 'TrailerPVMDetect', 'StaticPickupPVM': 'TrailerPVMDetect', 'invalid': 'modeDecision1'},
										autonomy={'PickupPVM': Autonomy.Off, 'SimPickupPVM': Autonomy.Off, 'StaticPickupPVM': Autonomy.Off, 'invalid': Autonomy.Off},
										remapping={'input_value': 'mode'})

			# x:265 y:70
			OperatableStateMachine.add('modeDecision2',
										DecisionState(outcomes=['PickupPVM' ,'SimPickupPVM', 'StaticPickupPVM','invalid'], conditions=lambda x: x if (x in ['PickupPVM' ,'SimPickupPVM', 'StaticPickupPVM']) else 'invalid'),
										transitions={'PickupPVM': 'CupOn', 'SimPickupPVM': 'log', 'StaticPickupPVM': 'CupOn', 'invalid': 'modeDecision2'},
										autonomy={'PickupPVM': Autonomy.Off, 'SimPickupPVM': Autonomy.Off, 'StaticPickupPVM': Autonomy.Off, 'invalid': Autonomy.Off},
										remapping={'input_value': 'mode'})

			# x:771 y:259
			OperatableStateMachine.add('modeDecision3',
										DecisionState(outcomes=['PickupPVM' ,'SimPickupPVM', 'StaticPickupPVM','invalid'], conditions=lambda x: x if (x in ['PickupPVM' ,'SimPickupPVM', 'StaticPickupPVM']) else 'invalid'),
										transitions={'PickupPVM': 'baseForward', 'SimPickupPVM': 'log', 'StaticPickupPVM': 'log', 'invalid': 'modeDecision3'},
										autonomy={'PickupPVM': Autonomy.Off, 'SimPickupPVM': Autonomy.Off, 'StaticPickupPVM': Autonomy.Off, 'invalid': Autonomy.Off},
										remapping={'input_value': 'mode'})

			# x:918 y:353
			OperatableStateMachine.add('startPlaceDetect',
										PublishHeader(seq=2, frame_id="solar_detect"),
										transitions={'done': 'checkPlaceSOlar'},
										autonomy={'done': Autonomy.Off})

			# x:940 y:470
			OperatableStateMachine.add('startPlaceSegment',
										PublishHeader(seq=2, frame_id="enable_yolov8"),
										transitions={'done': 'startPlaceDetect'},
										autonomy={'done': Autonomy.Off})

			# x:743 y:617
			OperatableStateMachine.add('test1025',
										self.use_behavior(test1025SM, 'test1025'),
										transitions={'finished': 'startPlaceSegment'},
										autonomy={'finished': Autonomy.Inherit})

			# x:501 y:350
			OperatableStateMachine.add('waitForward',
										WaitState(wait_time=2),
										transitions={'done': 'log'},
										autonomy={'done': Autonomy.Off})

			# x:26 y:395
			OperatableStateMachine.add('CupOff',
										self.use_behavior(CupOffSM, 'CupOff'),
										transitions={'finished': 'fromPlace2Pick'},
										autonomy={'finished': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
