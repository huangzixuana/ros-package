#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_states.handeye_action import HandeyeAction as comm_states__HandeyeAction
from comm_states.publish_string import PublishString
from comm_states.publisherheader import PublishHeader
from comm_states.site_manipulation import SiteManipulation
from flexbe_states.decision_state import DecisionState
from flexbe_states.subscriber_state import SubscriberState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Oct 31 2023
@author: Lei ZENG
'''
class HandEyePoseSM(Behavior):
	'''
	calibrate hand eye
	'''


	def __init__(self):
		super(HandEyePoseSM, self).__init__()
		self.name = 'HandEyePose'

		# parameters of this behavior
		self.add_parameter('idx', 1)
		self.add_parameter('auto', False)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:150 y:864
		_state_machine = OperatableStateMachine(outcomes=['finished'], input_keys=['sampler'], output_keys=['sampler'])
		_state_machine.userdata.move_group = None
		_state_machine.userdata.sampler = None
		_state_machine.userdata.auto = self.auto

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:122 y:44
			OperatableStateMachine.add('calibrateXPosePLan',
										SiteManipulation(pos=[0,0,0], quat=[0,0,0,1], target_frame="none", target_name='calibration_'+str(self.idx), axis_value=["none",0], pos_targets=[], trajectory_name='none', reference_frame="base_arm", end_effector_link="tool0", wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=False, cart_step_list=[3,11], step_factor=0.1, itp_norm=0.15, retry_num=3, cart_limit={}, if_execute=False, if_debug=False, planner_id='none', plan_time=2),
										transitions={'done': 'ifAuto', 'failed': 'calibrateXPosePLan'},
										autonomy={'done': Autonomy.High, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:131 y:151
			OperatableStateMachine.add('ifAuto',
										DecisionState(outcomes=['True', 'False'], conditions=lambda x: 'True' if x else 'False'),
										transitions={'True': 'calibrateXPose', 'False': 'informWeb'},
										autonomy={'True': Autonomy.Off, 'False': Autonomy.Off},
										remapping={'input_value': 'auto'})

			# x:129 y:507
			OperatableStateMachine.add('ifAuto2',
										DecisionState(outcomes=['True', 'False'], conditions=lambda x: 'True' if x else 'False'),
										transitions={'True': 'take_sample', 'False': 'informWeb2'},
										autonomy={'True': Autonomy.Off, 'False': Autonomy.Off},
										remapping={'input_value': 'auto'})

			# x:373 y:312
			OperatableStateMachine.add('ifNext',
										DecisionState(outcomes=['next', 'invalid'], conditions=lambda x: x.frame_id if (x.frame_id in ['next']) else 'invalid'),
										transitions={'next': 'calibrateXPose', 'invalid': 'informWeb'},
										autonomy={'next': Autonomy.Off, 'invalid': Autonomy.Off},
										remapping={'input_value': 'message1'})

			# x:377 y:648
			OperatableStateMachine.add('ifNext2',
										DecisionState(outcomes=['next', 'invalid'], conditions=lambda x: x.frame_id if (x.frame_id in ['next']) else 'invalid'),
										transitions={'next': 'take_sample', 'invalid': 'informWeb2'},
										autonomy={'next': Autonomy.Off, 'invalid': Autonomy.Off},
										remapping={'input_value': 'message2'})

			# x:374 y:155
			OperatableStateMachine.add('informWeb',
										PublishHeader(seq=1, frame_id="UI_handeye_arm:next?"),
										transitions={'done': 'subWeb'},
										autonomy={'done': Autonomy.Off})

			# x:376 y:510
			OperatableStateMachine.add('informWeb2',
										PublishHeader(seq=1, frame_id="UI_handeye_take:next?"),
										transitions={'done': 'subWeb2'},
										autonomy={'done': Autonomy.Off})

			# x:126 y:745
			OperatableStateMachine.add('pubClick',
										PublishString(name="easy_handeye", value="take"),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:557 y:200
			OperatableStateMachine.add('subWeb',
										SubscriberState(topic='trig', blocking=True, clear=True),
										transitions={'received': 'ifNext', 'unavailable': 'wait100ms'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message1'})

			# x:558 y:578
			OperatableStateMachine.add('subWeb2',
										SubscriberState(topic='trig', blocking=True, clear=True),
										transitions={'received': 'ifNext2', 'unavailable': 'wai100ms2'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message2'})

			# x:129 y:648
			OperatableStateMachine.add('take_sample',
										comm_states__HandeyeAction(client_action='take', algorithm='Andreff', if_write=False),
										transitions={'done': 'pubClick'},
										autonomy={'done': Autonomy.Off},
										remapping={'sampler_in': 'sampler', 'sampler_out': 'sampler'})

			# x:771 y:579
			OperatableStateMachine.add('wai100ms2',
										WaitState(wait_time=0.1),
										transitions={'done': 'subWeb2'},
										autonomy={'done': Autonomy.Off})

			# x:775 y:200
			OperatableStateMachine.add('wait100ms',
										WaitState(wait_time=0.1),
										transitions={'done': 'subWeb'},
										autonomy={'done': Autonomy.Off})

			# x:131 y:388
			OperatableStateMachine.add('wait3s',
										WaitState(wait_time=3),
										transitions={'done': 'ifAuto2'},
										autonomy={'done': Autonomy.High})

			# x:123 y:311
			OperatableStateMachine.add('calibrateXPose',
										SiteManipulation(pos=[0, 0, 0], quat=[0, 0, 0, 1], target_frame='none', target_name='calibration_'+str(self.idx), axis_value=['none', 0], pos_targets=[], trajectory_name='none', reference_frame='base_arm', end_effector_link='tool0', wait_time=0, v_factor=1, a_factor=1, t_factor=1.0, stay_level=False, cart_step_list=[3, 11], step_factor=0.1, itp_norm=0.15, retry_num=3, cart_limit={}, if_execute=True, if_debug=False, planner_id='none', plan_time=2),
										transitions={'done': 'wait3s', 'failed': 'calibrateXPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
