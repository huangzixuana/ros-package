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
from comm_behaviors.withdrawpvm_sm import WithdrawPVMSM
from flexbe_states.calculation_state import CalculationState
from flexbe_states.decision_state import DecisionState
from flexbe_states.log_state import LogState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jun 16 2023
@author: ZL
'''
class CommUninstallPVMSM(Behavior):
	'''
	uninstall pvm
	'''


	def __init__(self):
		super(CommUninstallPVMSM, self).__init__()
		self.name = 'CommUninstallPVM'

		# parameters of this behavior
		self.add_parameter('pvm_sum', 1)

		# references to used behaviors
		self.add_behavior(UninstallBracketPVMSM, 'UninstallBracketPVM')
		self.add_behavior(WithdrawPVMSM, 'WithdrawPVM')
		self.add_behavior(change_bzSM, 'change_bz')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished'])
		_state_machine.userdata.nav_goal = {'frame_id':'base_link', 'position':[0,0,0],'orientation':[0,0,0,1]}
		_state_machine.userdata.pvm_num = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('logr',
										LogState(text=self.pvm_sum, severity=2),
										transitions={'done': 'change_bz'},
										autonomy={'done': Autonomy.Off})

			# x:821 y:185
			OperatableStateMachine.add('WithdrawPVM',
										self.use_behavior(WithdrawPVMSM, 'WithdrawPVM',
											parameters={'install_mode': False}),
										transitions={'finished': 'counter'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'nav_goal': 'nav_goal'})

			# x:208 y:39
			OperatableStateMachine.add('change_bz',
										self.use_behavior(change_bzSM, 'change_bz'),
										transitions={'finished': 'wait5s'},
										autonomy={'finished': Autonomy.Inherit})

			# x:434 y:304
			OperatableStateMachine.add('comp',
										DecisionState(outcomes=['next', 'done'], conditions=lambda x: 'next' if x<self.pvm_sum else 'done'),
										transitions={'next': 'UninstallBracketPVM', 'done': 'finished'},
										autonomy={'next': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'input_value': 'pvm_num'})

			# x:845 y:306
			OperatableStateMachine.add('counter',
										CalculationState(calculation=lambda x: x+1),
										transitions={'done': 'log'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'pvm_num', 'output_value': 'pvm_num'})

			# x:635 y:305
			OperatableStateMachine.add('log',
										LogState(text='uninstall: +1', severity=Logger.REPORT_HINT),
										transitions={'done': 'comp'},
										autonomy={'done': Autonomy.Off})

			# x:437 y:76
			OperatableStateMachine.add('wait5s',
										WaitState(wait_time=8),
										transitions={'done': 'UninstallBracketPVM'},
										autonomy={'done': Autonomy.Off})

			# x:410 y:183
			OperatableStateMachine.add('UninstallBracketPVM',
										self.use_behavior(UninstallBracketPVMSM, 'UninstallBracketPVM'),
										transitions={'finished': 'WithdrawPVM'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'nav_goal': 'nav_goal'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
