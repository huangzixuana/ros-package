#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from std_msgs.msg import Header, String
from apriltag_ros.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
import rospy
import yaml
import tf2_ros
import tf
import time
import getpass
from PIL import Image
import numpy as np

'''
Created on 09.12.2019
@author: lei.zeng@tu-dortmund.de
'''


class TagRegister(EventState):
    '''
    tag register

    <= registered                   	   	   	   	 new tag registered
    <= covered                   	   	   	   	     existent tag updated
    <= failed                   	   	   	   	     failed

    '''

    def __init__(self):
        super(TagRegister, self).__init__(
            outcomes=['registered', 'covered', 'failed'])
        self._update_pub = ProxyPublisher({'flexbe/behavior_updating': String})
        self._pose_list = []
        self._tag_sub = ProxySubscriberCached(
            {'camera/tag_detections': AprilTagDetectionArray})
        self._odom_sub = ProxySubscriberCached({'odom': Odometry})
        self._buf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._buf)
        self._list_doc = {}
        self._special_area_path = self.relative_to_absolute_path(rospy.get_param(
            rospy.get_namespace()+"rosbridge_system/special_area_path", ''))

        map_path = self.relative_to_absolute_path("~/catkin_ws/dbparam/")
        img_display = Image.open(map_path + "map_display.png")
        img_tag = Image.open(map_path + "tag.png")
        width, height = img_display.size
        self._xmax, self._ymax = width/0.05, height/0.05

    def on_enter(self, userdata):
        update_msg = String()
        update_msg.data = self.name
        self._update_pub.publish('flexbe/behavior_updating', update_msg)

        self._pose_list = []
        self._list_doc = {}

    def robot_still(self):
        if self._odom_sub.has_msg('odom'):
            odom_msg = self._odom_sub.get_last_msg('odom')
            linear = odom_msg.twist.twist.linear.x
            omega = odom_msg.twist.twist.angular.z
            if linear == 0 and omega == 0:
                return True
            else:
                return False
        else:
            return False

    def get_tag_pose(self, tag_info):
        tag_name = 'tag_' + str(tag_info.id[0])
        tform = self._buf.lookup_transform("map",
                                           tag_name,
                                           rospy.Time(0),
                                           timeout=rospy.Duration(1))
        trans, rot = tform.transform.translation, tform.transform.rotation
        self._pose_list.append(
            ((trans.x, trans.y, trans.z),
             tf.transformations.euler_from_quaternion([rot.x, rot.y, rot.z, rot.w]))
        )

    def register_yaml(self, tag_info):
        tag_name = 'tag_' + str(tag_info.id[0])

        x_list = map(lambda f: f[0][0], self._pose_list)
        y_list = map(lambda f: f[0][1], self._pose_list)
        z_list = map(lambda f: f[0][2], self._pose_list)

        roll_list = map(lambda f: f[1][0], self._pose_list)
        pitch_list = map(lambda f: f[1][1], self._pose_list)
        yaw_list = map(lambda f: f[1][2], self._pose_list)

        position = [float(np.median(x_list)), float(
            np.median(y_list)), float(np.median(yaw_list))]
        orientation_np = tf.transformations.quaternion_from_euler(
            np.median(roll_list), np.median(pitch_list), np.median(yaw_list))
        orientation = [float(orientation_np[0]), float(orientation_np[1]), float(
            orientation_np[2]), float(orientation_np[3])]

        if 0 <= position[0] <= self._xmax and 0 <= position[1] <= self._ymax:
            tag_content = {
                'tag_id': tag_name,
                'size': tag_info.size[0],
                'frame': 'map',
                'position': position,
                'orientation': orientation,
                'time': time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time()))
            }
            self._list_doc[tag_name] = tag_content
            with open(self._special_area_path, "w") as f:
                yaml.dump(self._list_doc, f)
                Logger.loginfo(
                    "%s register at position ({0}, {1}, {2}) and  orientation ({3}, {4}, {5}, {6})"
                    .format(position[0], position[1], position[2],
                            orientation[0], orientation[1], orientation[2], orientation[3]) % tag_name)
            update_msg = String()
            update_msg.data = tag_name + " registered"
            self._update_pub.publish('flexbe/behavior_updating', update_msg)
            return True
        else:
            Logger.logwarn('Tag is out of map boundary')
            return False

    def execute(self, userdata):
        try:
            with open(self._special_area_path) as f:
                self._list_doc = yaml.safe_load(f)

        except Exception as e:
            Logger.logwarn('special_area file exception: %s' % str(e))

        if self._tag_sub.has_msg('camera/tag_detections') and self.robot_still:
            tags_msg = self._tag_sub.get_last_msg('camera/tag_detections')
            if len(tags_msg.detections) == 1:
                tag_info = tags_msg.detections[0]
                if abs(tag_info.pose.pose.pose.position.z) < 2.0 and abs(tag_info.pose.pose.pose.position.x) < 0.3 and tag_info.id[0] >= 100:
                    self.get_tag_pose(tag_info)
                    if len(self._pose_list) > 50:
                        tag_name = 'tag_' + str(tag_info.id[0])
                        if (tag_name not in self._list_doc):
                            try:
                                if self.register_yaml(tag_info):
                                    Logger.loginfo(
                                        'new %s register' % tag_name)
                                    return 'registered'
                                else:
                                    return 'failed'
                            except:
                                return 'failed'
                        elif (time.time() - time.mktime(time.strptime(self._list_doc[tag_name]['time'], "%Y-%m-%d %H:%M:%S"))) > 180:
                            try:
                                if self.register_yaml(tag_info):
                                    Logger.loginfo(
                                        '%s register: cover old data' % tag_name)
                                    return 'covered'
                                else:
                                    return 'failed'
                            except:
                                return 'failed'

                else:
                    self._pose_list = []
        else:
            self._pose_list = []

    def relative_to_absolute_path(self, relative_path):
        if relative_path[0] == '~':
            absolute_path = '/home/' + getpass.getuser() + relative_path[1:]
        else:
            absolute_path = relative_path
        return absolute_path

    def on_stop(self):
        self._odom_sub.unsubscribe_topic('odom')
        self._tag_sub.unsubscribe_topic('camera/tag_detections')

        # map_path = self.relative_to_absolute_path("~/catkin_ws/dbparam/")
        # img_display = Image.open(map_path + "map_display.png")
        # img_tag = Image.open(map_path + "tag.png")
        # _, height = img_display.size

        # with open(self._special_area_path, 'r') as stream:
        #     content = (yaml.safe_load(stream)).items()
        #     tags = [a for a in content if a[0][:3] == 'tag']
        #     for t in tags:
        #         x = int(t[1]['position'][0]/0.05)
        #         y = int(height - (t[1]['position'][1]/0.05))
        #         img_display.paste(img_tag, (x, y))

        # img_display.save(map_path + "map_display_tag.png")
