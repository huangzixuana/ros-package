#!/usr/bin/env python3
import json
import yaml
import getpass
import tf
import json
from shutil import copyfile
import time


def relative_to_absolute_path(relative_path):
    if relative_path[0] == '~':
        return '/home/' + getpass.getuser() + relative_path[1:]
    else:
        return relative_path


def wirte_camera_to_cfg():
    hand_eye_yaml_path = relative_to_absolute_path(
        "~/.ros/easy_handeye/my_eih_calib_eye_on_hand.yaml")
    with open(hand_eye_yaml_path) as handeye_file:
        handeye_dict = yaml.safe_load(handeye_file)
        print(handeye_dict)
        handeye_file.close()

        print("---")
        hand_eye_euler = tf.transformations.euler_from_quaternion([
            handeye_dict['transformation']['qx'], handeye_dict['transformation']['qy'],
            handeye_dict['transformation']['qz'], handeye_dict['transformation']['qw']
        ])
        hand_eye_euler = list(map(lambda i: round(i, 5), hand_eye_euler))
        print(hand_eye_euler)

        print("---")
        hand_eye_pos = [handeye_dict['transformation']['x'],
                        handeye_dict['transformation']['y'],  handeye_dict['transformation']['z']]
        print(hand_eye_pos)
        hand_eye_pos = list(map(lambda i: round(i, 5), hand_eye_pos))
        print(hand_eye_pos)

    print('===')
    cfg_path = relative_to_absolute_path("~/catkin_ws/dbparam/.cfg")
    with open(cfg_path) as cfg_file:
        cfg_contents = cfg_file.read()
        cfg_dict = json.loads(cfg_contents)
        print(cfg_dict)
        print('---')

        print(cfg_dict['comm'])
        copyfile(cfg_path, "/home/leizeng/catkin_ws/dbparam/.cfgcopy" + now_time())

        if 'camera' not in cfg_dict:
            print('no camera cfg')
            cfg_dict['camera'] = {}

        cfg_dict['camera']['$x'] = hand_eye_pos[0]
        cfg_dict['camera']['$y'] = hand_eye_pos[1]
        cfg_dict['camera']['$z'] = hand_eye_pos[2]

        cfg_dict['camera']['$roll'] = hand_eye_euler[0]
        cfg_dict['camera']['$pitch'] = hand_eye_euler[1]
        cfg_dict['camera']['$yaw'] = hand_eye_euler[2]
        print('cfg dict:')
        print(cfg_dict['camera'])

        cfg_write = json.dumps(cfg_dict, sort_keys=True, indent=2)

        with open(cfg_path, 'w') as f:
            f.write(cfg_write)
            f.close()


def now_time():
    return time.strftime("%Yy%mm%dd%Hh%Mm%Ss", time.localtime(time.time()))


def main():
    # hand_eye_yaml_path = relative_to_absolute_path()
    wirte_camera_to_cfg()


if __name__ == '__main__':
    main()
