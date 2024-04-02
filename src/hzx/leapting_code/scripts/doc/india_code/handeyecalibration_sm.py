#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from comm_behaviors.handeyeback_sm import HandEyeBackSM
from comm_behaviors.handeyepose_sm import HandEyePoseSM
from comm_states.handeye_action import HandeyeAction
from comm_states.publisherheader import PublishHeader
from comm_states.roslaunch_node import roslaunch_node
from comm_states.roslaunch_node_shut import roslaunch_node_shut
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
class HandEyeCalibrationSM(Behavior):
	'''
	calibrate hand eye
	'''


	def __init__(self):
		super(HandEyeCalibrationSM, self).__init__()
		self.name = 'HandEyeCalibration'

		# parameters of this behavior
		self.add_parameter('if_auto_all', False)

		# references to used behaviors
		self.add_behavior(HandEyeBackSM, 'HandEyeBack_10')
		self.add_behavior(HandEyeBackSM, 'HandEyeBack_15')
		self.add_behavior(HandEyeBackSM, 'HandEyeBack_16')
		self.add_behavior(HandEyeBackSM, 'HandEyeBack_5')
		self.add_behavior(HandEyePoseSM, 'HandEyePose')
		self.add_behavior(HandEyePoseSM, 'HandEyePose_10')
		self.add_behavior(HandEyePoseSM, 'HandEyePose_11')
		self.add_behavior(HandEyePoseSM, 'HandEyePose_12')
		self.add_behavior(HandEyePoseSM, 'HandEyePose_13')
		self.add_behavior(HandEyePoseSM, 'HandEyePose_14')
		self.add_behavior(HandEyePoseSM, 'HandEyePose_15')
		self.add_behavior(HandEyePoseSM, 'HandEyePose_16')
		self.add_behavior(HandEyePoseSM, 'HandEyePose_17')
		self.add_behavior(HandEyePoseSM, 'HandEyePose_18')
		self.add_behavior(HandEyePoseSM, 'HandEyePose_19')
		self.add_behavior(HandEyePoseSM, 'HandEyePose_2')
		self.add_behavior(HandEyePoseSM, 'HandEyePose_20')
		self.add_behavior(HandEyePoseSM, 'HandEyePose_21')
		self.add_behavior(HandEyePoseSM, 'HandEyePose_3')
		self.add_behavior(HandEyePoseSM, 'HandEyePose_4')
		self.add_behavior(HandEyePoseSM, 'HandEyePose_5')
		self.add_behavior(HandEyePoseSM, 'HandEyePose_6')
		self.add_behavior(HandEyePoseSM, 'HandEyePose_7')
		self.add_behavior(HandEyePoseSM, 'HandEyePose_8')
		self.add_behavior(HandEyePoseSM, 'HandEyePose_9')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1310 y:688
		_state_machine = OperatableStateMachine(outcomes=['finished'])
		_state_machine.userdata.pose_x = 1
		_state_machine.userdata.sampler = None
		_state_machine.userdata.move_group = None
		_state_machine.userdata.auto = self.if_auto_all

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:166 y:38
			OperatableStateMachine.add('launchCalibrate',
										roslaunch_node(cmd='roslaunch', pkg='bringup', launch_Node='calibrate.launch'),
										transitions={'done': 'wait10s'},
										autonomy={'done': Autonomy.Off},
										remapping={'output_value': 'output_value', 'shutdown_class': 'shutdown_class1'})

			# x:1530 y:187
			OperatableStateMachine.add('HandEyeBack_10',
										self.use_behavior(HandEyeBackSM, 'HandEyeBack_10',
											parameters={'idx': 10, 'auto': self.if_auto_all}),
										transitions={'finished': 'HandEyeBack_5'},
										autonomy={'finished': Autonomy.Inherit})

			# x:1542 y:66
			OperatableStateMachine.add('HandEyeBack_15',
										self.use_behavior(HandEyeBackSM, 'HandEyeBack_15',
											parameters={'idx': 15, 'auto': self.if_auto_all}),
										transitions={'finished': 'HandEyeBack_10'},
										autonomy={'finished': Autonomy.Inherit})

			# x:1343 y:61
			OperatableStateMachine.add('HandEyeBack_16',
										self.use_behavior(HandEyeBackSM, 'HandEyeBack_16',
											parameters={'idx': 16, 'auto': self.if_auto_all}),
										transitions={'finished': 'HandEyeBack_15'},
										autonomy={'finished': Autonomy.Inherit})

			# x:1315 y:184
			OperatableStateMachine.add('HandEyeBack_5',
										self.use_behavior(HandEyeBackSM, 'HandEyeBack_5',
											parameters={'idx': 5, 'auto': self.if_auto_all}),
										transitions={'finished': 'armPickupDetectPosePlan'},
										autonomy={'finished': Autonomy.Inherit})

			# x:158 y:226
			OperatableStateMachine.add('HandEyePose',
										self.use_behavior(HandEyePoseSM, 'HandEyePose',
											parameters={'idx': 1, 'auto': self.if_auto_all}),
										transitions={'finished': 'HandEyePose_2'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'sampler': 'sampler'})

			# x:362 y:221
			OperatableStateMachine.add('HandEyePose_10',
										self.use_behavior(HandEyePoseSM, 'HandEyePose_10',
											parameters={'idx': 10, 'auto': self.if_auto_all}),
										transitions={'finished': 'HandEyePose_11'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'sampler': 'sampler'})

			# x:574 y:61
			OperatableStateMachine.add('HandEyePose_11',
										self.use_behavior(HandEyePoseSM, 'HandEyePose_11',
											parameters={'idx': 11, 'auto': self.if_auto_all}),
										transitions={'finished': 'HandEyePose_12'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'sampler': 'sampler'})

			# x:573 y:192
			OperatableStateMachine.add('HandEyePose_12',
										self.use_behavior(HandEyePoseSM, 'HandEyePose_12',
											parameters={'idx': 12, 'auto': self.if_auto_all}),
										transitions={'finished': 'HandEyePose_13'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'sampler': 'sampler'})

			# x:576 y:314
			OperatableStateMachine.add('HandEyePose_13',
										self.use_behavior(HandEyePoseSM, 'HandEyePose_13',
											parameters={'idx': 13, 'auto': self.if_auto_all}),
										transitions={'finished': 'HandEyePose_14'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'sampler': 'sampler'})

			# x:578 y:443
			OperatableStateMachine.add('HandEyePose_14',
										self.use_behavior(HandEyePoseSM, 'HandEyePose_14',
											parameters={'idx': 14, 'auto': self.if_auto_all}),
										transitions={'finished': 'HandEyePose_15'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'sampler': 'sampler'})

			# x:571 y:596
			OperatableStateMachine.add('HandEyePose_15',
										self.use_behavior(HandEyePoseSM, 'HandEyePose_15',
											parameters={'idx': 15, 'auto': self.if_auto_all}),
										transitions={'finished': 'HandEyePose_16'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'sampler': 'sampler'})

			# x:791 y:600
			OperatableStateMachine.add('HandEyePose_16',
										self.use_behavior(HandEyePoseSM, 'HandEyePose_16',
											parameters={'idx': 16, 'auto': self.if_auto_all}),
										transitions={'finished': 'HandEyePose_17'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'sampler': 'sampler'})

			# x:785 y:451
			OperatableStateMachine.add('HandEyePose_17',
										self.use_behavior(HandEyePoseSM, 'HandEyePose_17',
											parameters={'idx': 17, 'auto': self.if_auto_all}),
										transitions={'finished': 'HandEyePose_18'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'sampler': 'sampler'})

			# x:796 y:318
			OperatableStateMachine.add('HandEyePose_18',
										self.use_behavior(HandEyePoseSM, 'HandEyePose_18',
											parameters={'idx': 18, 'auto': self.if_auto_all}),
										transitions={'finished': 'HandEyePose_19'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'sampler': 'sampler'})

			# x:796 y:189
			OperatableStateMachine.add('HandEyePose_19',
										self.use_behavior(HandEyePoseSM, 'HandEyePose_19',
											parameters={'idx': 19, 'auto': self.if_auto_all}),
										transitions={'finished': 'HandEyePose_20'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'sampler': 'sampler'})

			# x:165 y:340
			OperatableStateMachine.add('HandEyePose_2',
										self.use_behavior(HandEyePoseSM, 'HandEyePose_2',
											parameters={'idx': 2, 'auto': self.if_auto_all}),
										transitions={'finished': 'HandEyePose_3'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'sampler': 'sampler'})

			# x:798 y:59
			OperatableStateMachine.add('HandEyePose_20',
										self.use_behavior(HandEyePoseSM, 'HandEyePose_20',
											parameters={'idx': 20, 'auto': self.if_auto_all}),
										transitions={'finished': 'HandEyePose_21'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'sampler': 'sampler'})

			# x:1011 y:59
			OperatableStateMachine.add('HandEyePose_21',
										self.use_behavior(HandEyePoseSM, 'HandEyePose_21',
											parameters={'idx': 21, 'auto': self.if_auto_all}),
										transitions={'finished': 'wait3s'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'sampler': 'sampler'})

			# x:158 y:444
			OperatableStateMachine.add('HandEyePose_3',
										self.use_behavior(HandEyePoseSM, 'HandEyePose_3',
											parameters={'idx': 3, 'auto': self.if_auto_all}),
										transitions={'finished': 'HandEyePose_4'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'sampler': 'sampler'})

			# x:164 y:552
			OperatableStateMachine.add('HandEyePose_4',
										self.use_behavior(HandEyePoseSM, 'HandEyePose_4',
											parameters={'idx': 4, 'auto': self.if_auto_all}),
										transitions={'finished': 'HandEyePose_5'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'sampler': 'sampler'})

			# x:159 y:655
			OperatableStateMachine.add('HandEyePose_5',
										self.use_behavior(HandEyePoseSM, 'HandEyePose_5',
											parameters={'idx': 5, 'auto': self.if_auto_all}),
										transitions={'finished': 'HandEyePose_6'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'sampler': 'sampler'})

			# x:366 y:650
			OperatableStateMachine.add('HandEyePose_6',
										self.use_behavior(HandEyePoseSM, 'HandEyePose_6',
											parameters={'idx': 6, 'auto': self.if_auto_all}),
										transitions={'finished': 'HandEyePose_7'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'sampler': 'sampler'})

			# x:361 y:552
			OperatableStateMachine.add('HandEyePose_7',
										self.use_behavior(HandEyePoseSM, 'HandEyePose_7',
											parameters={'idx': 7, 'auto': self.if_auto_all}),
										transitions={'finished': 'HandEyePose_8'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'sampler': 'sampler'})

			# x:360 y:440
			OperatableStateMachine.add('HandEyePose_8',
										self.use_behavior(HandEyePoseSM, 'HandEyePose_8',
											parameters={'idx': 8, 'auto': self.if_auto_all}),
										transitions={'finished': 'HandEyePose_9'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'sampler': 'sampler'})

			# x:358 y:331
			OperatableStateMachine.add('HandEyePose_9',
										self.use_behavior(HandEyePoseSM, 'HandEyePose_9',
											parameters={'idx': 9, 'auto': self.if_auto_all}),
										transitions={'finished': 'HandEyePose_10'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'sampler': 'sampler'})

			# x:1012 y:376
			OperatableStateMachine.add('armPickupDetectPose',
										SiteManipulation(pos=[0, 0, 0], quat=[0, 0, 0, 1], target_frame='none', target_name='armPickupDetectPose', axis_value=['none', 0], pos_targets=[], reference_frame='base_arm', end_effector_link='tool0', v_factor=1, a_factor=1, if_execute=True, wait_time=3, stay_level=False, cart_step_list=[3, 8], retry_num=3, itp_norm=0, if_debug=True),
										transitions={'done': 'ComputeAndreff ', 'failed': 'armPickupDetectPose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:995 y:182
			OperatableStateMachine.add('armPickupDetectPosePlan',
										SiteManipulation(pos=[0, 0, 0], quat=[0, 0, 0, 1], target_frame='none', target_name='armPickupDetectPose', axis_value=['none', 0], pos_targets=[], reference_frame='base_arm', end_effector_link='tool0', v_factor=1, a_factor=1, if_execute=False, wait_time=0, stay_level=False, cart_step_list=[3, 8], retry_num=3, itp_norm=0, if_debug=False),
										transitions={'done': 'ifAuto', 'failed': 'armPickupDetectPosePlan'},
										autonomy={'done': Autonomy.High, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group'})

			# x:1031 y:284
			OperatableStateMachine.add('ifAuto',
										DecisionState(outcomes=['True', 'False'], conditions=lambda x: 'True' if x else 'False'),
										transitions={'True': 'armPickupDetectPose', 'False': 'informweb'},
										autonomy={'True': Autonomy.Off, 'False': Autonomy.Off},
										remapping={'input_value': 'auto'})

			# x:1227 y:374
			OperatableStateMachine.add('ifNext',
										DecisionState(outcomes=['next', 'invalid'], conditions=lambda x: x.frame_id if (x.frame_id in ['next']) else 'invalid'),
										transitions={'next': 'armPickupDetectPose', 'invalid': 'informweb'},
										autonomy={'next': Autonomy.Off, 'invalid': Autonomy.Off},
										remapping={'input_value': 'message'})

			# x:1222 y:282
			OperatableStateMachine.add('informweb',
										PublishHeader(seq=1, frame_id="UI_handeye_arm:next?"),
										transitions={'done': 'subWeb'},
										autonomy={'done': Autonomy.Off})

			# x:367 y:133
			OperatableStateMachine.add('launchApriltag',
										roslaunch_node(cmd='roslaunch', pkg='bringup', launch_Node='apriltag.launch'),
										transitions={'done': 'wait18s'},
										autonomy={'done': Autonomy.Off},
										remapping={'output_value': 'output_value', 'shutdown_class': 'shutdown_class2'})

			# x:1018 y:675
			OperatableStateMachine.add('shutApriltag',
										roslaunch_node_shut(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'shutdown_class': 'shutdown_class2'})

			# x:1017 y:577
			OperatableStateMachine.add('shutCalibrate',
										roslaunch_node_shut(),
										transitions={'done': 'shutApriltag'},
										autonomy={'done': Autonomy.Off},
										remapping={'shutdown_class': 'shutdown_class1'})

			# x:1386 y:278
			OperatableStateMachine.add('subWeb',
										SubscriberState(topic="trig", blocking=True, clear=True),
										transitions={'received': 'ifNext', 'unavailable': 'wait100ms'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:1540 y:286
			OperatableStateMachine.add('wait100ms',
										WaitState(wait_time=0.1),
										transitions={'done': 'subWeb'},
										autonomy={'done': Autonomy.Off})

			# x:371 y:35
			OperatableStateMachine.add('wait10s',
										WaitState(wait_time=10),
										transitions={'done': 'launchApriltag'},
										autonomy={'done': Autonomy.Off})

			# x:172 y:138
			OperatableStateMachine.add('wait18s',
										WaitState(wait_time=18),
										transitions={'done': 'HandEyePose'},
										autonomy={'done': Autonomy.Off})

			# x:1210 y:60
			OperatableStateMachine.add('wait3s',
										WaitState(wait_time=3),
										transitions={'done': 'HandEyeBack_16'},
										autonomy={'done': Autonomy.Off})

			# x:1021 y:472
			OperatableStateMachine.add('ComputeAndreff ',
										HandeyeAction(client_action='compute', algorithm='Andreff', if_write=True),
										transitions={'done': 'shutCalibrate'},
										autonomy={'done': Autonomy.Off},
										remapping={'sampler_in': 'sampler', 'sampler_out': 'sampler'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
