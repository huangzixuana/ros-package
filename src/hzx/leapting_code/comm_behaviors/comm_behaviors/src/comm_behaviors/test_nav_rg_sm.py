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
from comm_behaviors.startpvmdetect_sm import StartPVMDetectSM
from comm_behaviors.stoppvmdetect_sm import StopPVMDetectSM
from comm_states.realtime_goal import RealtimeGoal
from comm_states.site_navigation import SiteNavigation
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Aug 26 2024
@author: zcx
'''
class test_nav_rgSM(Behavior):
	'''
	test
	'''


	def __init__(self):
		super(test_nav_rgSM, self).__init__()
		self.name = 'test_nav_rg'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(StartPVMDetectSM, 'StartPVMDetect')
		self.add_behavior(StopPVMDetectSM, 'StopPVMDetect')
		self.add_behavior(change_bzSM, 'change_bz')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:458
		_state_machine = OperatableStateMachine(outcomes=['finished'])
		_state_machine.userdata.nav_goal = {'frame_id':'base_link', 'position':[0,0,0],'orientation':[0,0,0,1]}

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:63 y:40
			OperatableStateMachine.add('change_bz',
										self.use_behavior(change_bzSM, 'change_bz'),
										transitions={'finished': 'StartPVMDetect'},
										autonomy={'finished': Autonomy.Inherit})

			# x:282 y:329
			OperatableStateMachine.add('StopPVMDetect',
										self.use_behavior(StopPVMDetectSM, 'StopPVMDetect'),
										transitions={'finished': 'move'},
										autonomy={'finished': Autonomy.Inherit})

			# x:323 y:502
			OperatableStateMachine.add('move',
										SiteNavigation(site_name="", position=[0,0,0], orientation=[0,0,0,1], frame_id="map", base_link2map=True),
										transitions={'arrived': 'finished', 'canceled': 'move', 'failed': 'move'},
										autonomy={'arrived': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'nav_goal': 'nav_goal'})

			# x:313 y:198
			OperatableStateMachine.add('realtimeGoal',
										RealtimeGoal(position=[1,0.75,0], orientation=[0,0,0.7071,0.7071], frame_id='solar_link', source_frame='base_link', if_back=False, gap_pvm_pvm=1.52, gap_pvm_chassis=0.4, chassis_width=2.2, k_y=0.2, k_yaw=0.2),
										transitions={'done': 'StopPVMDetect'},
										autonomy={'done': Autonomy.Off},
										remapping={'goal': 'nav_goal'})

			# x:292 y:41
			OperatableStateMachine.add('StartPVMDetect',
										self.use_behavior(StartPVMDetectSM, 'StartPVMDetect'),
										transitions={'finished': 'realtimeGoal'},
										autonomy={'finished': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
