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
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jun 21 2023
@author: ZL
'''
class CupOnSM(Behavior):
	'''
	suction cup on
	'''


	def __init__(self):
		super(CupOnSM, self).__init__()
		self.name = 'CupOn'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:58 y:431
		_state_machine = OperatableStateMachine(outcomes=['finished'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:25
			OperatableStateMachine.add('cupon',
										PublishString(name="plc24_request", value="cup_on"),
										transitions={'done': 'checkpressure'},
										autonomy={'done': Autonomy.Off})

			# x:26 y:159
			OperatableStateMachine.add('checkpressure',
										VacuumPressure(action="cupon", threshold=500.0, frame=1),
										transitions={'done': 'att', 'timeout': 'checkpressure'},
										autonomy={'done': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:28 y:273
			OperatableStateMachine.add('att',
										SceneManager(action="attach", object_size=[2.298,1.164,0.15], frame_id="tool0", box_name="pvm1", box_position=[0,0,0]),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
