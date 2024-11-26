#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
import tf
from tf import TransformListener

class ListenTag(EventState):
    def __init__(self):
        super(ListenTag, self).__init__(outcomes=['done'])

        self._pose_sub = ProxySubscriberCached(
            {'tag_detections': AprilTagDetectionArray})
        self._pose_sub.subscribe('tag_detections', AprilTagDetectionArray,
                                 callback=self.tag_cb, buffered=False)
        self._received = False
        self._tag_msg = None
        
        self.tf1_lis = TransformListener()

    def tag_cb(self, msg):
        self._tag_msg = msg
        self._received = True

    def on_enter(self, userdata):
        self.tf1_lis.clear()
        self._received = False
        
    def execute(self, userdata):
        if rospy.has_param('/apriltag_detect/tag_bundles'):
            apriltag_param = rospy.get_param('/apriltag_detect/tag_bundles')
            bundles_name = []
            bundles_id = []
            for bi in apriltag_param:
                bundles_name.append(bi['name'])
                id_temp = []
                for d in bi['layout']:
                    id_temp.append(d['id'])
                bundles_id.append(id_temp)
        else:
            Logger.logwarn("%s: No info from apriltag_detect params" % self.name)
            return
        
        if self._received:
            detections_list = self._tag_msg.detections
            if (len(detections_list) < 1):
                Logger.logwarn("%s: No tag info from tag_detections" % self.name)
                return
            tag_id = [x for x in detections_list[0].id]
            if (len(tag_id) < 4):
                Logger.logwarn("%s: No enough tag id" % self.name)
                return
            bundle_in_topic = None
            for id_index in range(len(bundles_id)):
                if sorted(bundles_id[id_index]) == sorted(tag_id):
                    bundle_in_topic = bundles_name[id_index]
                    break
            if bundle_in_topic is None:
                Logger.logwarn("%s: Tag is wrong" % self.name)
                return
        else:
            Logger.logwarn("%s: No topic - tag_detections" % self.name)
            return

        bundle_frame_list = []
        frames_str = self.tf1_lis.allFramesAsString()
        str_list = frames_str.split('\n')
        del str_list[-1]
        for s in str_list:
            frame_list = s.split(' ')
            if frame_list[1].find("bundle") != -1 and not bundle_frame_list.__contains__(frame_list[1]):
                bundle_frame_list.append(frame_list[1])
            if frame_list[5][:-1].find("bundle") != -1 and not bundle_frame_list.__contains__(frame_list[5][:-1]):
                bundle_frame_list.append(frame_list[5][:-1])
        if bundle_in_topic not in bundle_frame_list:
            Logger.logwarn("%s: Bundle is different. Topic: %s. TF: %s" % (self.name, bundle_in_topic, bundle_frame_list))
            return
        else:
            return 'done'