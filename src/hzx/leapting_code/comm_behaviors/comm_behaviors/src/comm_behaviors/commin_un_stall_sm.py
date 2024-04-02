#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_behaviors.comminstallpvm_sm import CommInstallPVMSM
from comm_behaviors.communinstallpvm_sm import CommUninstallPVMSM
from comm_states.site_navigation import SiteNavigation
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jun 16 2023
@author: ZengLei
'''
class CommInUnstallSM(Behavior):
	'''
	install-uninstall
	'''


	def __init__(self):
		super(CommInUnstallSM, self).__init__()
		self.name = 'CommIn(Un)stall'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(CommInstallPVMSM, 'CommInstallPVM')
		self.add_behavior(CommUninstallPVMSM, 'CommUninstallPVM')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:581 y:100
		_state_machine = OperatableStateMachine(outcomes=['finished'])
		_state_machine.userdata.empty_goal = {}

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:65 y:74
			OperatableStateMachine.add('CommInstallPVM',
										self.use_behavior(CommInstallPVMSM, 'CommInstallPVM',
											parameters={'auto': True, 'pvm_sum': 2}),
										transitions={'finished': 'nav'},
										autonomy={'finished': Autonomy.Inherit})

			# x:61 y:349
			OperatableStateMachine.add('CommUninstallPVM',
										self.use_behavior(CommUninstallPVMSM, 'CommUninstallPVM',
											parameters={'pvm_sum': 2}),
										transitions={'finished': 'wait2s'},
										autonomy={'finished': Autonomy.Inherit})

			# x:79 y:166
			OperatableStateMachine.add('nav',
										SiteNavigation(site_name="", position=[1.1,0,0], orientation=[0,0,0,1], frame_id="base_link"),
										transitions={'arrived': 'wait3s', 'canceled': 'nav', 'failed': 'nav'},
										autonomy={'arrived': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'nav_goal': 'empty_goal'})

			# x:355 y:354
			OperatableStateMachine.add('navForward',
										SiteNavigation(site_name="", position=[0.8,0,0], orientation=[0,0,0,1], frame_id="base_link"),
										transitions={'arrived': 'wait2s', 'canceled': 'navForward', 'failed': 'navForward'},
										autonomy={'arrived': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'nav_goal': 'empty_goal'})

			# x:353 y:79
			OperatableStateMachine.add('wait2s',
										WaitState(wait_time=2),
										transitions={'done': 'CommInstallPVM'},
										autonomy={'done': Autonomy.Off})

			# x:74 y:249
			OperatableStateMachine.add('wait3s',
										WaitState(wait_time=3),
										transitions={'done': 'CommUninstallPVM'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
