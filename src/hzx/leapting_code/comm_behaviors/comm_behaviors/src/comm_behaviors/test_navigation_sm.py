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
from comm_behaviors.fromplacemid2pick_sm import fromPlaceMid2PickSM
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
		self.add_behavior(fromPlaceMid2PickSM, 'Container/fromPlaceMid2Pick')
		self.add_behavior(change_bzSM, 'change_bz')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:229 y:75
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.nav_goal = {}
		_state_machine.userdata.nav_1 = {}

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:458, x:415 y:487, x:521 y:516, x:330 y:458, x:430 y:447
		_sm_container_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['nav_goal'], conditions=[
										('finished', [('1', 'arrived'), ('fromPlaceMid2Pick', 'finished')]),
										('failed', [('1', 'canceled')]),
										('failed', [('1', 'failed')])
										])

		with _sm_container_0:
			# x:30 y:40
			OperatableStateMachine.add('fromPlaceMid2Pick',
										self.use_behavior(fromPlaceMid2PickSM, 'Container/fromPlaceMid2Pick'),
										transitions={'finished': 'finished'},
										autonomy={'finished': Autonomy.Inherit})

			# x:359 y:120
			OperatableStateMachine.add('1',
										SiteNavigation(site_name="", position=[1,0,0], orientation=[0,0,0,1], frame_id="base_link", base_link2map=False),
										transitions={'arrived': 'finished', 'canceled': 'failed', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'nav_goal': 'nav_goal'})



		with _state_machine:
			# x:0 y:52
			OperatableStateMachine.add('change_bz',
										self.use_behavior(change_bzSM, 'change_bz'),
										transitions={'finished': 'Container'},
										autonomy={'finished': Autonomy.Inherit})

			# x:425 y:336
			OperatableStateMachine.add('test-nav',
										SiteNavigation(site_name='', position=[1.3, 0, 0], orientation=[0, 0, 0, 1], frame_id='base_link', base_link2map=True),
										transitions={'arrived': 'finished', 'canceled': 'failed', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'nav_goal': 'nav_1'})

			# x:72 y:180
			OperatableStateMachine.add('Container',
										_sm_container_0,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'nav_goal': 'nav_goal'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
