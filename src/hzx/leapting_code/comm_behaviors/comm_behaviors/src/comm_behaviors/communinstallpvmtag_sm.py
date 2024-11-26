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
from comm_behaviors.uninstallbracketpvm_sm import UninstallBracketPVMSM
from comm_behaviors.withdrawpvmtag_sm import WithdrawPVMTagSM
from comm_states.manipulation_share import ManipulationShare
from comm_states.roslaunch_node import roslaunch_node
from comm_states.roslaunch_node_shut import roslaunch_node_shut
from comm_states.site_manipulation import SiteManipulation
from flexbe_states.calculation_state import CalculationState
from flexbe_states.decision_state import DecisionState
from flexbe_states.log_state import LogState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jun 16 2023
@author: ZL
'''
class CommUninstallPVMTagSM(Behavior):
	'''
	uninstall pvm
	'''


	def __init__(self):
		super(CommUninstallPVMTagSM, self).__init__()
		self.name = 'CommUninstallPVMTag'

		# parameters of this behavior
		self.add_parameter('pvm_sum', 4)

		# references to used behaviors
		self.add_behavior(UninstallBracketPVMSM, 'UninstallBracketPVM')
		self.add_behavior(WithdrawPVMTagSM, 'WithdrawPVMTag')
		self.add_behavior(change_bzSM, 'changeBZ')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:460 y:572
		_state_machine = OperatableStateMachine(outcomes=['finished'])
		_state_machine.userdata.nav_goal = {'frame_id':'base_link', 'position':[0,0,0],'orientation':[0,0,0,1]}
		_state_machine.userdata.pvm_num = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:55 y:28
			OperatableStateMachine.add('init',
										ManipulationShare(reference_frame="base_arm", end_effector_link="tool0"),
										transitions={'done': 'launchTag'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:817 y:182
			OperatableStateMachine.add('WithdrawPVMTag',
										self.use_behavior(WithdrawPVMTagSM, 'WithdrawPVMTag'),
										transitions={'finished': 'counter', 'failed': 'WithdrawPVMTag'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'nav_goal': 'nav_goal'})

			# x:213 y:179
			OperatableStateMachine.add('changeBZ',
										self.use_behavior(change_bzSM, 'changeBZ'),
										transitions={'finished': 'UninstallBracketPVM'},
										autonomy={'finished': Autonomy.Inherit})

			# x:434 y:304
			OperatableStateMachine.add('compare',
										DecisionState(outcomes=['next', 'done'], conditions=lambda x: 'next' if x<self.pvm_sum else 'done'),
										transitions={'next': 'UninstallBracketPVM', 'done': 'shutdown'},
										autonomy={'next': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'input_value': 'pvm_num'})

			# x:835 y:306
			OperatableStateMachine.add('counter',
										CalculationState(calculation=lambda x: x+1),
										transitions={'done': 'log2'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'pvm_num', 'output_value': 'pvm_num'})

			# x:221 y:63
			OperatableStateMachine.add('launchTag',
										roslaunch_node(cmd="roslaunch", pkg="bringup", launch_Node="apriltag.launch"),
										transitions={'done': 'changeBZ'},
										autonomy={'done': Autonomy.Off},
										remapping={'output_value': 'output_value', 'shutdown_class': 'shutdown_class'})

			# x:635 y:305
			OperatableStateMachine.add('log2',
										LogState(text='uninstall: +1', severity=Logger.REPORT_HINT),
										transitions={'done': 'compare'},
										autonomy={'done': Autonomy.Off})

			# x:407 y:414
			OperatableStateMachine.add('shutdown',
										roslaunch_node_shut(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'shutdown_class': 'shutdown_class'})

			# x:640 y:190
			OperatableStateMachine.add('top',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name="armTopPose", axis_value=["none",0], pos_targets=[], trajectory_name="none", reference_frame="base_arm", end_effector_link="tool0", wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=False, cart_step_list=[3,11], step_factor=0.1, itp_norm=0.15, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id="none", plan_time=2),
										transitions={'done': 'WithdrawPVMTag', 'failed': 'top'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:410 y:183
			OperatableStateMachine.add('UninstallBracketPVM',
										self.use_behavior(UninstallBracketPVMSM, 'UninstallBracketPVM',
											parameters={'unin_solar_z': 0.03, 'if_nav_back': False}),
										transitions={'finished': 'top'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'nav_goal': 'nav_goal'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
