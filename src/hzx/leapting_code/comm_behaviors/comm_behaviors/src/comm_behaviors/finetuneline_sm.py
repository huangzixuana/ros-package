#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_behaviors.finetune_sm import FinetuneSM
from comm_states.manipulation_share import ManipulationShare
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jan 30 2024
@author: zcx
'''
class FinetuneLineSM(Behavior):
	'''
	finetune according to lines in PVM
	'''


	def __init__(self):
		super(FinetuneLineSM, self).__init__()
		self.name = 'FinetuneLine'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(FinetuneSM, 'FinetuneX')
		self.add_behavior(FinetuneSM, 'FinetuneYaw')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:907 y:230
		_state_machine = OperatableStateMachine(outcomes=['done'])
		_state_machine.userdata.move_group = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:95 y:74
			OperatableStateMachine.add('armInit',
										ManipulationShare(reference_frame="base_arm", end_effector_link="tool0"),
										transitions={'done': 'wait5s'},
										autonomy={'done': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:344 y:221
			OperatableStateMachine.add('FinetuneYaw',
										self.use_behavior(FinetuneSM, 'FinetuneYaw',
											parameters={'adjust_element': "yaw"}),
										transitions={'finished': 'FinetuneX'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'move_group': 'move_group'})

			# x:357 y:74
			OperatableStateMachine.add('wait5s',
										WaitState(wait_time=5),
										transitions={'done': 'FinetuneYaw'},
										autonomy={'done': Autonomy.Off})

			# x:588 y:222
			OperatableStateMachine.add('FinetuneX',
										self.use_behavior(FinetuneSM, 'FinetuneX',
											parameters={'adjust_element': "x"}),
										transitions={'finished': 'done'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'move_group': 'move_group'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
