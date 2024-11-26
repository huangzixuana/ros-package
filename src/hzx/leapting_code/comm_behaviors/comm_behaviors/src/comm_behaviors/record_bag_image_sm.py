#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_states.record_bag import RecordBagState
from comm_states.record_image import RecordImageState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Sep 04 2024
@author: hzx
'''
class record_bag_imageSM(Behavior):
	'''
	record_bag_image
	'''


	def __init__(self):
		super(record_bag_imageSM, self).__init__()
		self.name = 'record_bag_image'

		# parameters of this behavior
		self.add_parameter('num', '0')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:396 y:47, x:157 y:254
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('recordbag',
										RecordBagState(topic_name='/livox/lidar', record_duration=15, fixed_path='/home/nvidia/hzx/bag/', bag_file_name=self.num+".bag"),
										transitions={'done': 'recordimage', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:202 y:40
			OperatableStateMachine.add('recordimage',
										RecordImageState(topic_name='/camera/image_raw', fixed_path='/home/nvidia/hzx/image/', image_file_name=self.num+".png"),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
