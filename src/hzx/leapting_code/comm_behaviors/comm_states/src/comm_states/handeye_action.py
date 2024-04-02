#!/usr/bin/env python
from flexbe_core import EventState, Logger
import easy_handeye_msgs as ehm
from easy_handeye.handeye_client import HandeyeClient

from easy_handeye.handeye_calibration import HandeyeCalibration, HandeyeCalibrationParameters
from easy_handeye.handeye_calibration_backend_opencv import HandeyeCalibrationBackendOpenCV
from easy_handeye.handeye_sampler import HandeyeSampler
import std_srvs
import time
import tf
import getpass
import yaml
import json
from shutil import copyfile

'''
Created on Nov 1, 2023
@author: lei.zeng@leapting.com
'''


class HandeyeAction(EventState):
    '''
    easy handeye actions

    -- client_action	       string            take(take_sample)/compute(compute_calibration and save))
    -- algorithm               string            'Tsai-Lenz'/'Park'/'Horaud'/'Andreff'/'Daniilidis'
    -- if_write                bool               if write result to cfg only when client_action=compute

    <= done                  transform determined 

    '''

    def __init__(self, client_action='take', algorithm='Andreff', if_write=False):
        super(HandeyeAction, self).__init__(
            outcomes=['done'], input_keys=['sampler_in'], output_keys=['sampler_out'])
        self._client_action = client_action
        # self._sampler = HandeyeSampler(handeye_parameters=self._parameters)
        self._algorithm = algorithm
        self._if_write = if_write

        self._calibration_backends = {
            'OpenCV': HandeyeCalibrationBackendOpenCV()}
        self._calibration_algorithm = 'OpenCV/' + algorithm
        self._last_calibration = None

    def on_enter(self, userdata):
        namespace = '/my_eih_calib_eye_on_hand/'
        self._parameters = HandeyeCalibrationParameters.init_from_parameter_server(
            namespace)
        if userdata.sampler_in == None:
            self._sampler = HandeyeSampler(handeye_parameters=self._parameters)
        else:
            self._sampler = userdata.sampler_in

        if self._client_action == 'take':
            Logger.loginfo('%s: taking sample...' % self.name)
            self.take_sample()
            Logger.loginfo('%s: done' % self.name)
        elif self._client_action == 'compute':
            Logger.loginfo('%s: computing...' % self.name)
            self.compute_calibration()
            Logger.loginfo('%s: done' % self.name)

            Logger.loginfo('%s: saving...' % self.name)
            self.save_calibration()
            try:
                copy_path = self._last_calibration.filename(
                )[:-5] + self._algorithm + self.now_time() + '.yaml'
                Logger.loginfo('%s: copy to %s' % (self.name, copy_path))
                copyfile(self._last_calibration.filename(), copy_path)
            except:
                pass
            Logger.loginfo('%s: done' % self.name)

            if self._if_write:
                self.wirte_camera_to_cfg()

    def execute(self, userdata):
        userdata.sampler_out = self._sampler
        return 'done'

    def retrieve_sample_list(self):
        ret = ehm.msg.SampleList()
        for s in self._sampler.get_samples():
            ret.camera_marker_samples.append(s['optical'].transform)
            ret.hand_world_samples.append(s['robot'].transform)
        return ret

    def take_sample(self):
        self._sampler.take_sample()
        return ehm.srv.TakeSampleResponse(self.retrieve_sample_list())

    def compute_calibration(self):
        samples = self._sampler.get_samples()
        Logger.loginfo('samples num: {}'.format(len(samples)))
        bckname, algname = self._calibration_algorithm.split('/')
        backend = self._calibration_backends[bckname]

        self._last_calibration = backend.compute_calibration(
            self._parameters, samples, algorithm=algname)
        ret = ehm.srv.ComputeCalibrationResponse()
        if self._last_calibration is None:
            Logger.logwarn('No valid calibration computed')
            ret.valid = False
            return ret
        ret.valid = True
        ret.calibration.eye_on_hand = self._last_calibration.parameters.eye_on_hand
        ret.calibration.transform = self._last_calibration.transformation
        return ret

    def save_calibration(self):
        if self._last_calibration:
            HandeyeCalibration.to_file(self._last_calibration)
            Logger.loginfo('Calibration saved to {}'.format(
                self._last_calibration.filename()))
        return std_srvs.srv.EmptyResponse()

    def wirte_camera_to_cfg(self):
        hand_eye_yaml_path = self.relative_to_absolute_path(
            "~/.ros/easy_handeye/my_eih_calib_eye_on_hand.yaml")
        cfg_path = self.relative_to_absolute_path("~/catkin_ws/dbparam/.cfg")
        try:
            with open(hand_eye_yaml_path) as handeye_file:
                handeye_dict = yaml.safe_load(handeye_file)
                print(handeye_dict)
                handeye_file.close()

            hand_eye_euler = tf.transformations.euler_from_quaternion([
                handeye_dict['transformation']['qx'], handeye_dict['transformation']['qy'],
                handeye_dict['transformation']['qz'], handeye_dict['transformation']['qw']
            ])
            hand_eye_euler = list(map(lambda i: round(i, 5), hand_eye_euler))
            hand_eye_pos = [handeye_dict['transformation']['x'],
                            handeye_dict['transformation']['y'],  handeye_dict['transformation']['z']]
            hand_eye_pos = list(map(lambda i: round(i, 5), hand_eye_pos))

            cfg_copy_path = self.relative_to_absolute_path(
                "~/.ros/easy_handeye/cfgcopy") + self.now_time()
            copyfile(cfg_path, cfg_copy_path)
            Logger.loginfo("%s: backup .cfg to %s" %
                           (self.name,  cfg_copy_path))
            with open(cfg_path) as cfg_file:
                cfg_contents = cfg_file.read()
                cfg_dict = json.loads(cfg_contents)
                cfg_file.close()
            if 'camera' not in cfg_dict:
                Logger.logwarn(
                    "%s: camera cfg not complete, please check it" % self.name)
                cfg_dict['camera'] = {}

            cfg_dict['camera']['$x'] = str(hand_eye_pos[0])
            cfg_dict['camera']['$y'] = str(hand_eye_pos[1])
            cfg_dict['camera']['$z'] = str(hand_eye_pos[2])
            cfg_dict['camera']['$roll'] = str(hand_eye_euler[0])
            cfg_dict['camera']['$pitch'] = str(hand_eye_euler[1])
            cfg_dict['camera']['$yaw'] = str(hand_eye_euler[2])

            cfg_write = json.dumps(cfg_dict, sort_keys=True, indent=2)
            with open(cfg_path, 'w') as f:
                f.write(cfg_write)
                f.close()

            Logger.loginfo("%s: save xyz-rpy to cfg successfully" % self.name)

        except:
            pass

    def relative_to_absolute_path(self, relative_path):
        if relative_path[0] == '~':
            return '/home/' + getpass.getuser() + relative_path[1:]
        else:
            return relative_path

    def now_time(self):
        return time.strftime("%Yy%mm%dd%Hh%Mm%Ss", time.localtime(time.time()))
