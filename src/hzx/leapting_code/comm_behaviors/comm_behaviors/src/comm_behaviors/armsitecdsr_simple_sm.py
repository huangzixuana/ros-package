#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_states.arm_site_cdsr_srdf_yaml import ArmSiteCdsrSrdfYaml
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Nov 21 2023
@author: zcx
'''
class armSiteCdsr_simpleSM(Behavior):
	'''
	simple armSiteCdsr
	'''


	def __init__(self):
		super(armSiteCdsr_simpleSM, self).__init__()
		self.name = 'armSiteCdsr_simple'

		# parameters of this behavior
		self.add_parameter('operation', 'add')
		self.add_parameter('operation_file', 'srdf')
		self.add_parameter('site_name', '')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:704 y:29, x:323 y:368, x:433 y:366, x:516 y:368
		_state_machine = OperatableStateMachine(outcomes=['done', 'failed', 'srdf_failed', 'yaml_failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:113 y:25
			OperatableStateMachine.add('wait2s',
										WaitState(wait_time=2),
										transitions={'done': 'arm_operate'},
										autonomy={'done': Autonomy.Off})

			# x:383 y:25
			OperatableStateMachine.add('arm_operate',
										ArmSiteCdsrSrdfYaml(srdf_path='~/catkin_ws/dbparam/arm3100_waypoints.srdf', yaml_path='~/catkin_ws/dbparam/arm3100_waypoints.yaml', source_frame='base_arm', target_frame='tool0', operation=self.operation, operation_file=self.operation_file, site_name=self.site_name),
										transitions={'done': 'done', 'failed': 'failed', 'srdf_failed': 'srdf_failed', 'yaml_failed': 'yaml_failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'srdf_failed': Autonomy.Off, 'yaml_failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
