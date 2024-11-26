#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_behaviors.change_bz_sm import change_bzSM
from comm_behaviors.change_teb_sm import change_tebSM
from comm_states.site_navigation import SiteNavigation
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jan 19 2024
@author: Lei ZENG
'''
class nav_changeSM(Behavior):
	'''
	navigation change bz and teb if timeout
	'''


	def __init__(self):
		super(nav_changeSM, self).__init__()
		self.name = 'nav_change'

		# parameters of this behavior
		self.add_parameter('wait_time', 10)

		# references to used behaviors
		self.add_behavior(change_bzSM, 'change_bz')
		self.add_behavior(change_tebSM, 'change_teb')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:195 y:286
		_state_machine = OperatableStateMachine(outcomes=['finished'], input_keys=['nav_goal'])
		_state_machine.userdata.nav_goal = {}

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365, x:530 y:365, x:630 y:365
		_sm_1_0 = ConcurrencyContainer(outcomes=['finished', 'failed', 'timeout'], input_keys=['nav_goal'], conditions=[
										('finished', [('navi', 'arrived')]),
										('failed', [('navi', 'canceled')]),
										('failed', [('navi', 'failed')]),
										('timeout', [('wait10s', 'done')])
										])

		with _sm_1_0:
			# x:113 y:161
			OperatableStateMachine.add('navi',
										SiteNavigation(site_name='', position=[0, 0, 0], orientation=[0, 0, 0, 1], frame_id='map', base_link2map=True),
										transitions={'arrived': 'finished', 'canceled': 'failed', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'nav_goal': 'nav_goal'})

			# x:277 y:158
			OperatableStateMachine.add('wait10s',
										WaitState(wait_time=self.wait_time),
										transitions={'done': 'timeout'},
										autonomy={'done': Autonomy.Off})



		with _state_machine:
			# x:161 y:35
			OperatableStateMachine.add('1',
										_sm_1_0,
										transitions={'finished': 'finished', 'failed': '1', 'timeout': 'change_teb'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'timeout': Autonomy.Inherit},
										remapping={'nav_goal': 'nav_goal'})

			# x:434 y:33
			OperatableStateMachine.add('change_bz',
										self.use_behavior(change_bzSM, 'change_bz'),
										transitions={'finished': '1'},
										autonomy={'finished': Autonomy.Inherit})

			# x:435 y:162
			OperatableStateMachine.add('change_teb',
										self.use_behavior(change_tebSM, 'change_teb'),
										transitions={'finished': 'change_bz'},
										autonomy={'finished': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
