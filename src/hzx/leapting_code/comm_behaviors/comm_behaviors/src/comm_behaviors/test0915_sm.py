#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_behaviors.commpickuppvm_sm import CommPickupPVMSM
from comm_behaviors.commplacepvm_sm import CommPlacePVMSM
from comm_behaviors.fromplace2pick_sm import fromPlace2PickSM
from comm_behaviors.withdrawpvm_sm import WithdrawPVMSM
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Aug 25 2023
@author: ZL
'''
class test0915SM(Behavior):
	'''
	install one PVM
	'''


	def __init__(self):
		super(test0915SM, self).__init__()
		self.name = 'test0915'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(CommPickupPVMSM, 'CommPickupPVM')
		self.add_behavior(CommPlacePVMSM, 'CommPlacePVM')
		self.add_behavior(WithdrawPVMSM, 'WithdrawPVM')
		self.add_behavior(fromPlace2PickSM, 'fromPlace2Pick')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:163 y:567
		_state_machine = OperatableStateMachine(outcomes=['finished'])
		_state_machine.userdata.nav_goal = {}

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:167 y:44
			OperatableStateMachine.add('CommPickupPVM',
										self.use_behavior(CommPickupPVMSM, 'CommPickupPVM',
											parameters={'mode': "StaticPickupPVM", 'pick_ideal_y': -0.107}),
										transitions={'finished': 'CommPlacePVM'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'nav_goal': 'nav_goal'})

			# x:167 y:164
			OperatableStateMachine.add('CommPlacePVM',
										self.use_behavior(CommPlacePVMSM, 'CommPlacePVM',
											parameters={'mode': "PlacePVM", 'install_gap': -5, 'place_solar_x': -0.03, 'place_solar_z': 0.0, 'place_ideal_z': 0.01}),
										transitions={'finished': 'wait5s'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'nav_goal': 'nav_goal'})

			# x:404 y:155
			OperatableStateMachine.add('WithdrawPVM',
										self.use_behavior(WithdrawPVMSM, 'WithdrawPVM'),
										transitions={'finished': 'CommPickupPVM'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'nav_goal': 'nav_goal'})

			# x:151 y:432
			OperatableStateMachine.add('fromPlace2Pick',
										self.use_behavior(fromPlace2PickSM, 'fromPlace2Pick'),
										transitions={'finished': 'finished'},
										autonomy={'finished': Autonomy.Inherit})

			# x:184 y:273
			OperatableStateMachine.add('wait5s',
										WaitState(wait_time=5),
										transitions={'done': 'WithdrawPVM'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
