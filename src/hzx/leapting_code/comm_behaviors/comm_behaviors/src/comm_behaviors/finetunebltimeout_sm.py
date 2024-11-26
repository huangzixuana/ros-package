#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_behaviors.finetunebl_sm import FinetuneBLSM
from comm_states.publisherheader import PublishHeader
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Mar 14 2024
@author: zcx
'''
class FinetuneBLTimeoutSM(Behavior):
	'''
	finetune
	'''


	def __init__(self):
		super(FinetuneBLTimeoutSM, self).__init__()
		self.name = 'FinetuneBLTimeout'

		# parameters of this behavior
		self.add_parameter('adjust_element', 'yaw')
		self.add_parameter('yline_gap', 36)
		self.add_parameter('adjust_topic', 'res_line')
		self.add_parameter('stop_element', 'line')

		# references to used behaviors
		self.add_behavior(FinetuneBLSM, 'Container/FinetuneBL')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:174 y:425, x:431 y:427
		_state_machine = OperatableStateMachine(outcomes=['finished', 'timeout'], output_keys=['adjust_goal'])
		_state_machine.userdata.adjust_goal = {'pos':[0,0,0], 'quat':[0,0,0,1], 'target_frame':'tool0'}

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365
		_sm_container_0 = ConcurrencyContainer(outcomes=['finished', 'timeout'], output_keys=['adjust_goal'], conditions=[
										('finished', [('FinetuneBL', 'finished')]),
										('timeout', [('wait15s', 'done')])
										])

		with _sm_container_0:
			# x:156 y:108
			OperatableStateMachine.add('FinetuneBL',
										self.use_behavior(FinetuneBLSM, 'Container/FinetuneBL',
											parameters={'adjust_element': self.adjust_element, 'yline_gap': self.yline_gap, 'adjust_topic': self.adjust_topic, 'stop_element': self.stop_element}),
										transitions={'finished': 'finished'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'adjust_goal': 'adjust_goal'})

			# x:377 y:110
			OperatableStateMachine.add('wait15s',
										WaitState(wait_time=15),
										transitions={'done': 'timeout'},
										autonomy={'done': Autonomy.Off})



		with _state_machine:
			# x:141 y:134
			OperatableStateMachine.add('Container',
										_sm_container_0,
										transitions={'finished': 'finished', 'timeout': 'stopAdjust'},
										autonomy={'finished': Autonomy.Inherit, 'timeout': Autonomy.Inherit},
										remapping={'adjust_goal': 'adjust_goal'})

			# x:398 y:133
			OperatableStateMachine.add('stopAdjust',
										PublishHeader(seq=0, frame_id="enable_"+self.stop_element),
										transitions={'done': 'timeout'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
