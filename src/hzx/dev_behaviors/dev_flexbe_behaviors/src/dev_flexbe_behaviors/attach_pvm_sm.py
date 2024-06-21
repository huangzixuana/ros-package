#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from dev_flexbe_states.scene_manager import SceneManager
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Oct 17 2023
@author: hzx
'''
class attach_pvmSM(Behavior):
	'''
	attach_pvm
	'''


	def __init__(self):
		super(attach_pvmSM, self).__init__()
		self.name = 'attach_pvm'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:37 y:130
			OperatableStateMachine.add('attach',
										SceneManager(action="attach", object_size=[2.338,1.194,0.1], frame_id="tool0", box_name="pvm", box_position=[0,0,0]),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:282 y:48
			OperatableStateMachine.add('de',
										SceneManager(action="dttach", object_size=[2.278,1.134,0.035], frame_id="tool0", box_name="pvm", box_position=[0,0,0]),
										transitions={'done': 'wait'},
										autonomy={'done': Autonomy.Off})

			# x:76 y:46
			OperatableStateMachine.add('wa',
										WaitState(wait_time=1),
										transitions={'done': 'de'},
										autonomy={'done': Autonomy.Off})

			# x:191 y:217
			OperatableStateMachine.add('wait',
										WaitState(wait_time=1),
										transitions={'done': 'attach'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
