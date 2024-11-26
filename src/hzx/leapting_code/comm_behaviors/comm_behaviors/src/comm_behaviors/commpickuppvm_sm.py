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
from comm_behaviors.fromplacemid2pick_sm import fromPlaceMid2PickSM
from comm_behaviors.trailerpvmdetect_sm import TrailerPVMDetectSM
from comm_states.manipulation_share import ManipulationShare
from comm_states.site_manipulation import SiteManipulation
from comm_states.site_navigation import SiteNavigation
from comm_states.update_tool0_pose import CalculateNewToolPose as comm_states__CalculateNewToolPose
from dev_flexbe_states.scene_manager import SceneManager as dev_flexbe_states__SceneManager
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
		self.add_parameter('pick_solar_x', 0.000001)
		self.add_parameter('pick_solar_y', 0.05)
		self.add_parameter('pick_solar_z', 0.02)
		self.add_parameter('pick_ideal_x', 2.1)
		self.add_parameter('pick_ideal_y', -0.006)
		self.add_parameter('pick_ideal_z', 0.0)
		self.add_parameter('pick_ideal_er', 0.006)
		self.add_parameter('pick_ideal_ep', -0.011)
		self.add_parameter('pick_ideal_ey', 1.559)
		self.add_parameter('pick_tol_x', -1.0)
		self.add_parameter('pick_tol_y', -1.0)
		self.add_parameter('pick_tol_z', -1.0)
		self.add_parameter('pick_tol_er', 0.2)
		self.add_parameter('pick_tol_ep', 0.2)
		self.add_parameter('pick_tol_ey', 0.2)
		self.add_parameter('detect_option', 'line')

		# references to used behaviors
		self.add_behavior(CupOffSM, 'CupOff')
		self.add_behavior(CupOnSM, 'CupOn')
		self.add_behavior(TrailerPVMDetectSM, 'TrailerPVMDetect')
		self.add_behavior(fromPlaceMid2PickSM, 'fromPlaceMid2Pick')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:811 y:444
		_state_machine = OperatableStateMachine(outcomes=['finished'], input_keys=['nav_goal', 'pose_msg_in', 'robot_pose', 'tool_pose'], output_keys=['pose_msg_pick', 'robot_pose', 'tool_pose'])
		_state_machine.userdata.nav_goal = {'frame_id':'base_link', 'position':[0,0,0],'orientation':[0,0,0,1]}
		_state_machine.userdata.mode = self.mode
		_state_machine.userdata.detect_option = self.detect_option
		_state_machine.userdata.empty_goal = {}
		_state_machine.userdata.pose_msg_in = None
		_state_machine.userdata.pose_msg_pick = None
		_state_machine.userdata.robot_pose = None
		_state_machine.userdata.tool_pose = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:62 y:308, x:546 y:246, x:11 y:125, x:522 y:137, x:683 y:94, x:380 y:505, x:291 y:431
		_sm_armnav_0 = ConcurrencyContainer(outcomes=['finished', 'nav_failed', 'arm_failed'], input_keys=['nav_goal', 'move_group'], conditions=[
										('finished', [('baseForward', 'arrived'), ('armReturn', 'done')]),
										('nav_failed', [('baseForward', 'canceled')]),
										('nav_failed', [('baseForward', 'failed')]),
										('arm_failed', [('armReturn', 'failed')])
										])

		with _sm_armnav_0:
			# x:316 y:161
			OperatableStateMachine.add('baseForward',
										SiteNavigation(site_name='', position=[0, 0, 0], orientation=[0, 0, 0, 1], frame_id='map', base_link2map=False),
										transitions={'arrived': 'finished', 'canceled': 'nav_failed', 'failed': 'nav_failed'},
										autonomy={'arrived': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'nav_goal': 'nav_goal'})

			# x:91 y:150
			OperatableStateMachine.add('armReturn',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="none", axis_value=["none",0], pos_targets=["armPlaceDetectPose","armPickupDetectPose"], trajectory_name="none", reference_frame="base_arm", end_effector_link="tool0", wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=True, cart_step_list=[3,19], step_factor=0.1, itp_norm=0, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id="none", plan_time=2),
										transitions={'done': 'finished', 'failed': 'arm_failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})



		with _state_machine:
			# x:32 y:54
			OperatableStateMachine.add('armInit',
										ManipulationShare(reference_frame='base_arm', end_effector_link='tool0'),
										transitions={'done': 'modeDecision1'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:31 y:291
			OperatableStateMachine.add('CupOff',
										self.use_behavior(CupOffSM, 'CupOff'),
										transitions={'finished': 'storeToolPose'},
										autonomy={'finished': Autonomy.Inherit})

			# x:462 y:218
			OperatableStateMachine.add('CupOn',
										self.use_behavior(CupOnSM, 'CupOn'),
										transitions={'finished': 'armUp150cm'},
										autonomy={'finished': Autonomy.Inherit})

			# x:247 y:73
			OperatableStateMachine.add('TrailerPVMDetect',
										self.use_behavior(TrailerPVMDetectSM, 'TrailerPVMDetect',
											parameters={'pick_solar_x': self.pick_solar_x, 'pick_solar_y': self.pick_solar_y, 'pick_solar_z': self.pick_solar_z, 'pick_ideal_x': self.pick_ideal_x, 'pick_ideal_y': self.pick_ideal_y, 'pick_ideal_z': self.pick_ideal_z, 'pick_ideal_er': self.pick_ideal_er, 'pick_ideal_ep': self.pick_ideal_ep, 'pick_ideal_ey': self.pick_ideal_ey, 'pick_tol_x': self.pick_tol_x, 'pick_tol_y': self.pick_tol_y, 'pick_tol_z': self.pick_tol_z, 'pick_tol_er': self.pick_tol_er, 'pick_tol_ep': self.pick_tol_ep, 'pick_tol_ey': self.pick_tol_ey, 'detect_option': self.detect_option}),
										transitions={'finished': 'modeDecision2'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'nav_goal': 'nav_goal', 'pose_msg_in': 'pose_msg_in', 'pose_msg_pick': 'pose_msg_pick'})

			# x:34 y:574
			OperatableStateMachine.add('armPlaceUP10cm',
										SiteManipulation(pos=[0, 0, 0.1], quat=[0, 0, 0, 1], target_frame='tool0', target_name='none', axis_value=['none', 0], pos_targets=[], trajectory_name='none', reference_frame='base_arm', end_effector_link='tool0', wait_time=1, v_factor=1, a_factor=1, t_factor=1.0, stay_level=True, cart_step_list=[1, 11], step_factor=0.001, itp_norm=0, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id='none', plan_time=2),
										transitions={'done': 'ArmNav', 'failed': 'armPlaceUP10cm'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:760 y:199
			OperatableStateMachine.add('armTopPose',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armTopPose", axis_value=["none",0], pos_targets=[], trajectory_name="none", reference_frame="base_arm", end_effector_link="tool0", wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=False, cart_step_list=[3,11], step_factor=0.1, itp_norm=0.15, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id="none", plan_time=2),
										transitions={'done': 'attach', 'failed': 'armTopPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:777 y:75
			OperatableStateMachine.add('armUp150cm',
										SiteManipulation(pos=[0,0,1.5], quat=[0,0,0,1], target_frame="tool0", target_name="none", axis_value=["none",0], pos_targets=[], trajectory_name="none", reference_frame="base_arm", end_effector_link="tool0", wait_time=0, v_factor=0.3, a_factor=0.3, t_factor=1.0, stay_level=True, cart_step_list=[3,11], step_factor=0.1, itp_norm=0, retry_num=3, cart_limit={'z_max': 1.45}, if_execute=True, if_debug=False, planner_id="none", plan_time=2),
										transitions={'done': 'armTopPose', 'failed': 'armUp150cm'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:774 y:317
			OperatableStateMachine.add('attach',
										dev_flexbe_states__SceneManager(action="attach", object_size=[2.338,1.194,0.15], frame_id="tool0", box_name="pvm1", box_position=[0,0,0]),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:377 y:429
			OperatableStateMachine.add('baseForward2',
										SiteNavigation(site_name="", position=[0,0,0], orientation=[0,0,0,1], frame_id="map", base_link2map=True),
										transitions={'arrived': 'waitForward', 'canceled': 'baseForward2', 'failed': 'baseForward2'},
										autonomy={'arrived': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'nav_goal': 'nav_goal'})

			# x:601 y:413
			OperatableStateMachine.add('fromPlaceMid2Pick',
										self.use_behavior(fromPlaceMid2PickSM, 'fromPlaceMid2Pick'),
										transitions={'finished': 'waitForward'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'move_group': 'move_group'})

			# x:322 y:498
			OperatableStateMachine.add('log',
										LogState(text="to nav", severity=Logger.REPORT_HINT),
										transitions={'done': 'baseForward2'},
										autonomy={'done': Autonomy.Off})

			# x:261 y:228
			OperatableStateMachine.add('logState',
										LogState(text="next pickup", severity=2),
										transitions={'done': 'TrailerPVMDetect'},
										autonomy={'done': Autonomy.Off})

			# x:36 y:153
			OperatableStateMachine.add('modeDecision1',
										DecisionState(outcomes=['PickupPVM' ,'SimPickupPVM', 'StaticPickupPVM','invalid'], conditions=lambda x: x if (x in ['PickupPVM' ,'SimPickupPVM', 'StaticPickupPVM']) else 'invalid'),
										transitions={'PickupPVM': 'CupOff', 'SimPickupPVM': 'logState', 'StaticPickupPVM': 'logState', 'invalid': 'modeDecision1'},
										autonomy={'PickupPVM': Autonomy.Off, 'SimPickupPVM': Autonomy.Off, 'StaticPickupPVM': Autonomy.Off, 'invalid': Autonomy.Off},
										remapping={'input_value': 'mode'})

			# x:473 y:72
			OperatableStateMachine.add('modeDecision2',
										DecisionState(outcomes=['PickupPVM' ,'SimPickupPVM', 'StaticPickupPVM','invalid'], conditions=lambda x: x if (x in ['PickupPVM' ,'SimPickupPVM', 'StaticPickupPVM']) else 'invalid'),
										transitions={'PickupPVM': 'CupOn', 'SimPickupPVM': 'armUp150cm', 'StaticPickupPVM': 'CupOn', 'invalid': 'modeDecision2'},
										autonomy={'PickupPVM': Autonomy.Off, 'SimPickupPVM': Autonomy.Off, 'StaticPickupPVM': Autonomy.Off, 'invalid': Autonomy.Off},
										remapping={'input_value': 'mode'})

			# x:28 y:432
			OperatableStateMachine.add('storeToolPose',
										comm_states__CalculateNewToolPose(action="pose_in", offset_x=-1.2, offset_y=0.0, offset_z=0.12),
										transitions={'done': 'armPlaceUP10cm'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group', 'robot_pose': 'robot_pose', 'tool_pose': 'tool_pose'})

			# x:232 y:328
			OperatableStateMachine.add('waitForward',
										WaitState(wait_time=0.1),
										transitions={'done': 'logState'},
										autonomy={'done': Autonomy.Off})

			# x:223 y:572
			OperatableStateMachine.add('ArmNav',
										_sm_armnav_0,
										transitions={'finished': 'waitForward', 'nav_failed': 'log', 'arm_failed': 'fromPlaceMid2Pick'},
										autonomy={'finished': Autonomy.Inherit, 'nav_failed': Autonomy.Inherit, 'arm_failed': Autonomy.Inherit},
										remapping={'nav_goal': 'nav_goal', 'move_group': 'move_group'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
