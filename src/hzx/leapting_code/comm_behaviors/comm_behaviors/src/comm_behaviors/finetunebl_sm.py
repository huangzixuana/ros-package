#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_states.line_adjust import LineAdjust
from comm_states.publisherheader import PublishHeader
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Mar 14 2024
@author: zcx
'''
class FinetuneBLSM(Behavior):
	'''
	finetune
	'''


	def __init__(self):
		super(FinetuneBLSM, self).__init__()
		self.name = 'FinetuneBL'

		# parameters of this behavior
		self.add_parameter('adjust_element', 'yaw')
		self.add_parameter('yline_gap', 36)
		self.add_parameter('adjust_topic', 'res_line')
		self.add_parameter('stop_element', 'line')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:913 y:423
		_state_machine = OperatableStateMachine(outcomes=['finished'], output_keys=['adjust_goal'])
		_state_machine.userdata.adjust_goal = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:289 y:112
			OperatableStateMachine.add('startAdjust',
										PublishHeader(seq=self.yline_gap, frame_id="adjust_"+self.adjust_element),
										transitions={'done': 'info'},
										autonomy={'done': Autonomy.Off})

			# x:469 y:251
			OperatableStateMachine.add('stopAdjust',
										PublishHeader(seq=0, frame_id="enable_"+self.stop_element),
										transitions={'done': 'startAdjust'},
										autonomy={'done': Autonomy.Off})

			# x:629 y:114
			OperatableStateMachine.add('stopAdjust2',
										PublishHeader(seq=0, frame_id="enable_"+self.stop_element),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:293 y:251
			OperatableStateMachine.add('wait2s',
										WaitState(wait_time=2),
										transitions={'done': 'startAdjust'},
										autonomy={'done': Autonomy.Off})

			# x:470 y:113
			OperatableStateMachine.add('info',
										LineAdjust(adjust_topic=self.adjust_topic, timeout=60, fresh_time=3.0, ideal=[0, 0, 0, 0, 0, 0], tolerance=[-1, -1, -1, -1, -1, -1], max_z=0.15),
										transitions={'done': 'stopAdjust2', 'timeout': 'stopAdjust'},
										autonomy={'done': Autonomy.Off, 'timeout': Autonomy.Off},
										remapping={'adjust_goal': 'adjust_goal'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
