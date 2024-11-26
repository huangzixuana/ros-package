#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_states.publish_string import PublishString
from comm_states.scene_manager import SceneManager
from comm_states.vacuum_pressure import VacuumPressure
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jun 21 2023
@author: ZL
'''
class CupOffSM(Behavior):
	'''
	suction cup off
	'''


	def __init__(self):
		super(CupOffSM, self).__init__()
		self.name = 'CupOff'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:46 y:436
		_state_machine = OperatableStateMachine(outcomes=['finished'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('cupoff',
										PublishString(name="plc24_request", value="cup_off"),
										transitions={'done': 'checkpressure'},
										autonomy={'done': Autonomy.Off})

			# x:26 y:305
			OperatableStateMachine.add('det',
										SceneManager(action="detach", object_size=[2.278,1.134,0.035], frame_id="tool0", box_name="pvm1", box_position=[0,0,0]),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:198 y:168
			OperatableStateMachine.add('wait1s',
										WaitState(wait_time=1),
										transitions={'done': 'det'},
										autonomy={'done': Autonomy.Off})

			# x:25 y:166
			OperatableStateMachine.add('checkpressure',
										VacuumPressure(action="cupoff", threshold=20.0, frame=2),
										transitions={'done': 'det', 'timeout': 'checkpressure'},
										autonomy={'done': Autonomy.Off, 'timeout': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
