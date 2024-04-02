#!/usr/bin/env python3
"""
Author: lei.zeng@leapting.com
"""
import os
import copy
import re
import json
import yaml
import getpass
import xml.etree.ElementTree as ET
import rospy
import tf2_ros
import zlib
import numpy as np
from rospkg import RosPack
from std_msgs.msg import String, Header, Bool
from geometry_msgs.msg import Quaternion, Vector3
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from flexbe_msgs.msg import BEStatus
from flexbe_core import BehaviorLibrary


class flexbeBridgeNode(object):
    def __init__(self):
        self.diagnostics_pub = rospy.Publisher('/diagnostics',
                                               DiagnosticArray,
                                               queue_size=10)

        self.rospack = RosPack()
        self.flexbe_list_pub = rospy.Publisher(
            'flexbe/behavior_list', String, queue_size=1, latch=True)
        self.flexbe_site_pub = rospy.Publisher(
            'flexbe/site_collection', String, queue_size=1, latch=True)
        self.behavior_pkg = rospy.get_param("~behavior_pkg", '')
        self.flexbe_site_path = relative_to_absolute_path(
            rospy.get_param("~waypoints_path", "~/catkin_ws/dbparam/flexbe_waypoints.yaml"))

        self.behavior_dict = {}
        behavior_pkg_list = []
        if self.behavior_pkg.find(':') < 0:
            behavior_pkg_list.append(self.behavior_pkg)
        else:
            divide_idx = [-1]
            for m in re.finditer(':', self.behavior_pkg):
                divide_idx.append(m.start())
            divide_idx.append(len(self.behavior_pkg))
            for i in range(len(divide_idx)-1):
                behavior_pkg_list.append(
                    (self.behavior_pkg[divide_idx[i]+1:divide_idx[i+1]]))
        self.manifest_path_list = []
        for p in behavior_pkg_list:
            self.manifest_path_list.append(
                self.rospack.get_path(p)+"/manifest")

        self.flexbe_site = {}
        if not os.path.isfile(self.flexbe_site_path):
            with open(self.flexbe_site_path, 'w') as fp:
                pass
            with open(self.flexbe_site_path, "w") as f:
                self.flexbe_site = {'initial_pose': {'frame_id': 'map',
                                                     'pose': {'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1},
                                                              'position': {'x': 0, 'y': 0, 'z': 0}
                                                              }
                                                     },
                                    'charge_point': {'frame_id': 'map',
                                                     'pose': {'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1},
                                                              'position': {'x': 0, 'y': 0, 'z': 0}
                                                              }
                                                     }
                                    }
                yaml.dump(self.flexbe_site, f)
                rospy.loginfo('flexbe site file %s created with default site' %
                              self.flexbe_site_path)

        rospy.Subscriber('trig',
                         Header,
                         self.trig_cb,
                         queue_size=10)

        self.flexbe_code_dict = {
            0: 'STARTED',
            1: 'FINISHED',
            2: 'FAILED',
            4: 'LOCKED',
            5: 'WAITING',
            6: 'SWITCHING',
            10: 'WARNING',
            11: 'ERROR',
            20: 'READY'
        }

        BeLib = BehaviorLibrary()
        self.check_num_dict = {}
        self.diag_dict = {'behavior': {'level': 0, 'name': '', 'msg': ''},
                          'state': {'name': '', 'pause': False}}
        for key in BeLib._behavior_lib:
            be_filepath_new = BeLib.get_sourcecode_filepath(key)
            with open(be_filepath_new, "r") as f:
                be_content_new = f.read()
                behavior_checksum = self.to_int16(
                    zlib.adler32(be_content_new.encode()))
                self.check_num_dict[
                    behavior_checksum] = BeLib._behavior_lib[key]['name']

        rospy.Subscriber('flexbe/status',
                         BEStatus,
                         self.flexbe_status_cb,
                         queue_size=10)
        rospy.Subscriber('flexbe/command/pause',
                         Bool,
                         self.flexbe_pause_cb,
                         queue_size=2)
        rospy.Subscriber('flexbe/behavior_update',
                         String,
                         self.flexbe_state_cb,
                         queue_size=10)

        rospy.Timer(rospy.Duration(1), self.time_cb, oneshot=False)

        self.flexbe_list_publish()
        self.flexbe_site_publish()

    def to_int16(self, n):
        # return np.array([n], 'int16')[0]
        return (np.array([n]).astype('int16'))[0]

    def time_cb(self, event):
        self.diag_dict['state']['level'] = 0
        if self.diag_dict['behavior']['level'] == 1:
            if self.diag_dict['state']['pause']:
                self.diag_dict['state']['level'] = 2
            else:
                self.diag_dict['state']['level'] = 1
        # L0: free, L1: run, L2:paused
        # M: state name: bahavior name
        state_diag_msg = diag_array_gen("task_state", self.diag_dict['state']['level'],
                                        "%s:%s" % (self.diag_dict['state']['name'], self.diag_dict['behavior']['name']))

        # L0: free, L1: run, L2:error
        # M: behavior name: bahavior state
        beh_diag_msg = diag_array_gen("task_behavior", self.diag_dict['behavior']['level'],
                                      "%s:%s" % (self.diag_dict['behavior']['name'], self.diag_dict['behavior']['msg']))
        state_diag_msg.status.append(beh_diag_msg.status[0])
        self.diagnostics_pub.publish(state_diag_msg)

    def flexbe_pause_cb(self, msg):
        if msg.data:
            self.diag_dict['state']['pause'] = True
        else:
            self.diag_dict['state']['pause'] = False

    def flexbe_state_cb(self, state_msg):
        self.diag_dict['state']['name'] = state_msg.data

    def flexbe_status_cb(self, status_msg):
        self.diag_dict['behavior']['level'] = 0
        self.diag_dict['behavior']['msg'] = ''
        if self.flexbe_code_dict[status_msg.code] in ['FINISHED', 'FAILED', 'READY']:
            self.diag_dict['state']['name'] = ''
            if self.flexbe_code_dict[status_msg.code] in ['FINISHED', 'FAILED']:
                self.diag_dict['behavior']['name'] = self.check_num_dict[self.to_int16(
                    status_msg.behavior_id)]
                if status_msg.args == ['preempted']:
                    self.diag_dict['behavior']['msg'] = 'preempted'
                else:
                    self.diag_dict['behavior']['msg'] = self.flexbe_code_dict[status_msg.code]
        elif self.flexbe_code_dict[status_msg.code] == 'STARTED':

            self.diag_dict['behavior']['level'] = 1
            self.diag_dict['behavior']['name'] = self.check_num_dict[self.to_int16(
                status_msg.behavior_id)]
        elif self.flexbe_code_dict[status_msg.code] == 'ERROR':
            rospy.logerr('flexbe status is ERROR')
            self.diag_dict['behavior']['level'] = 2

    def flexbe_list_publish(self):
        self.behavior_dict = {}
        for behavior_path in self.manifest_path_list:
            if not os.path.exists(behavior_path):
                rospy.logwarn("path %s not exits" % behavior_path)
                continue
            for b in os.listdir(behavior_path):
                f = behavior_path + '/' + b
                b_name = str(ET.parse(f).getroot().attrib['name'])
                self.behavior_dict[b_name] = {
                    "name": b[:-4],
                    "tag": ET.parse(f).getroot().find('tagstring').text,
                    "description": ET.parse(f).getroot().find('description').text
                }
        list_msg = String()
        list_msg.data = json.dumps(self.behavior_dict, sort_keys=True)
        self.flexbe_list_pub.publish(list_msg)

    def flexbe_site_publish(self):
        with open(self.flexbe_site_path) as site_f:
            self.flexbe_site = yaml.safe_load(site_f)
        msg = String()
        msg.data = json.dumps(
            {'flexbe_site': self.flexbe_site}, sort_keys=True)
        self.flexbe_site_pub.publish(msg)

    def trig_cb(self, msg):
        if msg.frame_id[:13] == 'flexbe_delete':
            self.flexbe_task_remove(msg.frame_id[14:])
            self.flexbe_list_publish()
        elif msg.frame_id[:11] == 'create_site':
            with open(self.flexbe_site_path, 'r') as f:
                list_doc = yaml.safe_load(f)
                list_doc_copy = copy.deepcopy(list_doc)
            if msg.frame_id[12:] not in list_doc:
                try:
                    with open(self.flexbe_site_path, 'w') as f:
                        site_pose, site_quat = get_tf_pose("map", "base_link")
                        list_doc[msg.frame_id[12:]] = {
                            'frame_id': 'map',
                            'pose': {
                                        'orientation': {'x': site_quat.x, 'y': site_quat.y, 'z': site_quat.z, 'w': site_quat.w},
                                        'position': {'x': site_pose.x, 'y': site_pose.y, 'z': site_pose.z}
                            }
                        }
                        yaml.dump(list_doc, f)
                        rospy.loginfo("flexbe site %s is created " %
                                      msg.frame_id[12:])
                except Exception as e:
                    with open(self.flexbe_site_path, 'w') as f:
                        yaml.dump(list_doc_copy, f)
                    rospy.logwarn(
                        "exception while flexbe site creation: %s" % str(e))
            else:
                rospy.logwarn('%s already exists in flexbe waypoint' %
                              msg.frame_id[12:])
            self.flexbe_site_publish()
        elif msg.frame_id[:11] == 'delete_site':
            self.delete_dict_item(
                self.flexbe_site_path, msg.frame_id[12:])
            self.flexbe_site_publish()
        elif msg.seq == 201:
            try:
                edit_site = json.loads(msg.frame_id)
            except Exception as e:
                rospy.logwarn(
                    "exception while json loading: %s" % str(e))
                return
            with open(self.flexbe_site_path, 'r') as f:
                list_doc = yaml.safe_load(f)
                list_doc_copy = copy.deepcopy(list_doc)
                list_doc.update(edit_site)
            try:
                with open(self.flexbe_site_path, 'w') as f:
                    # yaml.dump(list_doc, f)
                    yaml_doc = yaml.safe_dump(
                        list_doc, default_flow_style=False)
                    f.write(yaml_doc)
                    rospy.loginfo("flexbe waypoint is updated")
            except Exception as e:
                with open(self.flexbe_site_path, 'w') as f:
                    yaml.dump(list_doc_copy, f)
                rospy.logwarn(
                    "exception while flexbe site edit: %s" % str(e))
            self.flexbe_site_publish()

    def flexbe_task_remove(self, beh_name):
        if beh_name not in self.behavior_dict:
            rospy.loginfo('flexbe behavior %s not exists' % beh_name)
            return
        for behavior_path in reversed(self.manifest_path_list):
            xml_path = '%s/%s.xml' % (behavior_path,
                                      self.behavior_dict[beh_name]['name'])
            pkg_e = behavior_path.index('_behaviors')
            ss = behavior_path[:pkg_e]
            sm_path = '%ssrc/%s_behaviors/%s_sm.py' % (
                behavior_path[:-8], behavior_path[ss.rindex('/')+1: pkg_e], self.behavior_dict[beh_name]['name'])
            if os.path.isfile(xml_path) and os.path.isfile(sm_path):
                os.remove(xml_path)
                os.remove(sm_path)
                rospy.loginfo('flexbe behavior %s is removed' % beh_name)
                break
            else:
                continue

    def delete_dict_item(self, file_path, k):
        with open(file_path, 'r') as f:
            list_doc = yaml.safe_load(f)
            list_doc_copy = copy.deepcopy(list_doc)
        try:
            if k in list_doc:
                with open(file_path, 'w') as f:
                    del list_doc[k]
                    yaml.dump(list_doc, f)
            else:
                pass
            rospy.loginfo('%s deleted' % k)

        except Exception as e:
            with open(file_path, 'w') as f:
                yaml.dump(list_doc_copy, f)
            rospy.logwarn("exception to delete %s: %s" % (k, str(e)))


def relative_to_absolute_path(relative_path):
    if relative_path[0] == '~':
        return '/home/' + getpass.getuser() + relative_path[1:]
    else:
        return relative_path


def get_tf_pose(parent_frame, child_frame):
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    try:
        tform = buf.lookup_transform(parent_frame,
                                     child_frame,
                                     rospy.Time(0),
                                     timeout=rospy.Duration(4))
        pos_trans = tform.transform.translation
        pose_quat = tform.transform.rotation
        return (pos_trans, pose_quat)
    except:
        rospy.logerr('TF Exception to get transform between %s and %s' %
                     (parent_frame, child_frame))
        q = Quaternion()
        q.w = 1
        return (Vector3(), q)


def diag_array_gen(d_name, d_level, d_msg):
    da_msg = DiagnosticArray()
    da_msg.header.stamp = rospy.Time.now()
    status_msg = DiagnosticStatus()
    status_msg.name = d_name
    status_msg.level = d_level
    status_msg.message = d_msg
    da_msg.status.append(status_msg)
    return da_msg


def main():
    rospy.init_node('flexbe_bridge')
    flexbeBridgeNode()
    try:
        rospy.spin()
    except:
        print("Shutting down...")


if __name__ == '__main__':
    main()
