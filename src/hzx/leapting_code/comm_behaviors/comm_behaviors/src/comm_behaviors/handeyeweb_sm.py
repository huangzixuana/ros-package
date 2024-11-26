#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_states.publisherheader import PublishHeader
from flexbe_states.decision_state import DecisionState
from flexbe_states.subscriber_state import SubscriberState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Oct 31 2023
@author: zcx
'''
class HandEyeWebSM(Behavior):
	'''
	web
	'''


	def __init__(self):
		super(HandEyeWebSM, self).__init__()
		self.name = 'HandEyeWeb'

		# parameters of this behavior
		self.add_parameter('web_info', '')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:399 y:460
		_state_machine = OperatableStateMachine(outcomes=['finished'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:374 y:155
			OperatableStateMachine.add('informWeb',
										PublishHeader(seq=1, frame_id=self.web_info),
										transitions={'done': 'subWeb'},
										autonomy={'done': Autonomy.Off})

			# x:557 y:200
			OperatableStateMachine.add('subWeb',
										SubscriberState(topic='trig', blocking=True, clear=True),
										transitions={'received': 'ifNext', 'unavailable': 'wait100ms'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message1'})

			# x:775 y:200
			OperatableStateMachine.add('wait100ms',
										WaitState(wait_time=0.1),
										transitions={'done': 'subWeb'},
										autonomy={'done': Autonomy.Off})

			# x:373 y:312
			OperatableStateMachine.add('ifNext',
										DecisionState(outcomes=['next', 'invalid'], conditions=lambda x: x.frame_id if (x.frame_id in ['next']) else 'invalid'),
										transitions={'next': 'finished', 'invalid': 'informWeb'},
										autonomy={'next': Autonomy.Off, 'invalid': Autonomy.Off},
										remapping={'input_value': 'message1'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
