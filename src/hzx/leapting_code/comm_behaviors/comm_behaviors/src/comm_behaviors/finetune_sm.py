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
from comm_states.site_manipulation_ud import SiteManipulationUD
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Mar 14 2024
@author: zcx
'''
class FinetuneSM(Behavior):
	'''
	finetune
	'''


	def __init__(self):
		super(FinetuneSM, self).__init__()
		self.name = 'Finetune'

		# parameters of this behavior
		self.add_parameter('adjust_element', 'yaw')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:913 y:423
		_state_machine = OperatableStateMachine(outcomes=['finished'], input_keys=['move_group'])
		_state_machine.userdata.move_group = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:289 y:112
			OperatableStateMachine.add('startAdjust',
										PublishHeader(seq=2, frame_id="adjust_"+self.adjust_element),
										transitions={'done': 'info'},
										autonomy={'done': Autonomy.Off})

			# x:470 y:113
			OperatableStateMachine.add('info',
										LineAdjust(timeout=60, fresh_time=3.0, ideal=[0, 0, 0, 0, 0, 0], tolerance=[-1, -1, -1, -1, -1, -1], max_z=0.07),
										transitions={'done': 'stopAdjust2', 'timeout': 'stopAdjust'},
										autonomy={'done': Autonomy.Off, 'timeout': Autonomy.Off},
										remapping={'adjust_goal': 'adjust_goal', 'goal_z': 'goal_z'})

			# x:470 y:250
			OperatableStateMachine.add('stopAdjust',
										PublishHeader(seq=0, frame_id="enable_adjust"),
										transitions={'done': 'wait2s2'},
										autonomy={'done': Autonomy.Off})

			# x:629 y:114
			OperatableStateMachine.add('stopAdjust2',
										PublishHeader(seq=0, frame_id="enable_adjust"),
										transitions={'done': 'finetune'},
										autonomy={'done': Autonomy.Off})

			# x:849 y:252
			OperatableStateMachine.add('wait2s',
										WaitState(wait_time=0.1),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:293 y:251
			OperatableStateMachine.add('wait2s2',
										WaitState(wait_time=2),
										transitions={'done': 'startAdjust'},
										autonomy={'done': Autonomy.Off})

			# x:832 y:113
			OperatableStateMachine.add('finetune',
										SiteManipulationUD(reference_frame='base_arm', end_effector_link='tool0', v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=True, cart_step_list=[3, 11], step_factor=0.1, retry_num=3, itp_norm=0, if_debug=False, cart_limit={}),
										transitions={'done': 'wait2s', 'failed': 'finetune'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'mani_goal': 'adjust_goal'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
