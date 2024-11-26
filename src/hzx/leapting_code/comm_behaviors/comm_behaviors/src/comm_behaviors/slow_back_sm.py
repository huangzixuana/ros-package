#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from agv_flexbe_behaviors.change_bz_sm import change_bzSM
from comm_states.reconfigure_state import ReconfigureState as comm_states__ReconfigureState
from comm_states.site_navigation import SiteNavigation as comm_states__SiteNavigation
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jul 13 2023
@author: ZL
'''
class slow_backSM(Behavior):
	'''
	test
	'''


	def __init__(self):
		super(slow_backSM, self).__init__()
		self.name = 'slow_back'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(change_bzSM, 'change_bz')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:33 y:454
		_state_machine = OperatableStateMachine(outcomes=['finished'])
		_state_machine.userdata.empty_goal = {}

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:42 y:92
			OperatableStateMachine.add('change_bz',
										self.use_behavior(change_bzSM, 'change_bz'),
										transitions={'finished': 'vel1_slow'},
										autonomy={'finished': Autonomy.Inherit})

			# x:241 y:382
			OperatableStateMachine.add('vel1_high',
										comm_states__ReconfigureState(client="move_base/BZPlannerROS", parameter="max_vel_x", value=0.25),
										transitions={'done': 'vel2_high'},
										autonomy={'done': Autonomy.Off})

			# x:252 y:72
			OperatableStateMachine.add('vel1_slow',
										comm_states__ReconfigureState(client="move_base/BZPlannerROS", parameter="max_vel_x", value=0.08),
										transitions={'done': 'vel2_slow'},
										autonomy={'done': Autonomy.Off})

			# x:188 y:470
			OperatableStateMachine.add('vel2_high',
										comm_states__ReconfigureState(client="move_base/BZPlannerROS", parameter="min_vel_x", value=0.15),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:260 y:143
			OperatableStateMachine.add('vel2_slow',
										comm_states__ReconfigureState(client="move_base/BZPlannerROS", parameter="min_vel_x", value=0.05),
										transitions={'done': 'back10cm'},
										autonomy={'done': Autonomy.Off})

			# x:264 y:307
			OperatableStateMachine.add('wait2s',
										WaitState(wait_time=2),
										transitions={'done': 'vel1_high'},
										autonomy={'done': Autonomy.Off})

			# x:274 y:234
			OperatableStateMachine.add('back10cm',
										comm_states__SiteNavigation(site_name="", position=[-0.1,0,0], orientation=[0,0,0,1], frame_id="base_link"),
										transitions={'arrived': 'wait2s', 'canceled': 'back10cm', 'failed': 'back10cm'},
										autonomy={'arrived': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'nav_goal': 'empty_goal'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
