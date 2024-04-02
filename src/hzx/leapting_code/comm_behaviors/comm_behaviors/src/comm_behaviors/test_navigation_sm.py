#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_states.site_navigation import SiteNavigation
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jan 19 2024
@author: Lei ZENG
'''
class testnavigationSM(Behavior):
	'''
	test
	'''


	def __init__(self):
		super(testnavigationSM, self).__init__()
		self.name = 'test-navigation'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:229 y:75
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.nav_goal = {}

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:47 y:146
			OperatableStateMachine.add('test-nav',
										SiteNavigation(site_name='', position=[1, 0, 0], orientation=[0, 0, 0, 1], frame_id='base_link', base_link2map=False),
										transitions={'arrived': 'finished', 'canceled': 'failed', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'nav_goal': 'nav_goal'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
