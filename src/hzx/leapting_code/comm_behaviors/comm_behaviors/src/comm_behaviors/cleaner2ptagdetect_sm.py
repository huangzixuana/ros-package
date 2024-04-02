#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_states.listen_state import ListenState
from comm_states.site_manipulation import SiteManipulation
from flexbe_states.log_state import LogState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Mar 05 2024
@author: Lei ZENG
'''
class Cleaner2pTagDetectSM(Behavior):
	'''
	apriltag detection and arm action
	'''


	def __init__(self):
		super(Cleaner2pTagDetectSM, self).__init__()
		self.name = 'Cleaner2pTagDetect'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:

		# O 312 138 
		# log placeholder: todo

		# O 305 330 
		# may have TF fresh warning, todo



	def create(self):
		# x:232 y:646, x:448 y:482
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.move_group = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:199 y:132
			OperatableStateMachine.add('startTagDetect',
										LogState(text='todo', severity=2),
										transitions={'done': 'waitTagDetect'},
										autonomy={'done': Autonomy.Off})

			# x:201 y:325
			OperatableStateMachine.add('checkTag',
										ListenState(target_frame='', source_frame='tool0', timeout=60, fresh_time=3.0, ideal=[0, 0, 0, 0, 0, 0], tolerance=[-1, -1, -1, -1, -1, -1]),
										transitions={'done': 'armTagDockPose', 'timeout': 'failed'},
										autonomy={'done': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:35 y:450
			OperatableStateMachine.add('wait1s',
										WaitState(wait_time=1),
										transitions={'done': 'armTagDockPose'},
										autonomy={'done': Autonomy.Off})

			# x:200 y:243
			OperatableStateMachine.add('waitTagDetect',
										WaitState(wait_time=2),
										transitions={'done': 'checkTag'},
										autonomy={'done': Autonomy.Off})

			# x:196 y:449
			OperatableStateMachine.add('armTagDockPose',
										SiteManipulation(pos=[0, 0, 0], quat=[0, 0, 0, 1], target_frame='none', target_name='none', axis_value=['none', 0], pos_targets=[], reference_frame='base_arm', end_effector_link='tool0', v_factor=1, a_factor=1, if_execute=True, wait_time=0, stay_level=False, cart_step_list=[3, 11], retry_num=3, itp_norm=0.15, if_debug=False, cart_limit={}),
										transitions={'done': 'finished', 'failed': 'wait1s'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
