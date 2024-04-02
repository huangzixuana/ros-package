#!/usr/bin/env python3
"""
Author: lei.zeng@leapting.com, chenxin.zou@leapting.com
"""
import getpass
import numpy as np
import math
import os
import cv2
import rospy
from sensor_msgs.msg import Image
import torch
from ultralytics import YOLO
from cv_bridge import CvBridge
import time
from std_msgs.msg import Header, String
import parsing_yolo8_model_name
from dynamic_reconfigure.server import Server
from yolov8_result.cfg import Yolov8Config
import base64

def cv2_to_base64(cv_image):
    img_msg = String()
    img_encode = cv2.imencode('.png', cv_image)[1]
    base64_data_str = str(base64.b64encode(img_encode))[2:-1]
    img_msg.data = base64_data_str
    return img_msg

class Yolov8Process(object):
    def __init__(self):
        model_ab_path = relative_to_absolute_path(
            rospy.get_param("~model_folder_path", "~/catkin_ws/src/yolov8_result/model"))
        try:
            self.model_info = parsing_yolo8_model_name.parsing_name(
                model_ab_path)
        except Exception as e:
            rospy.logerr('Yolov8Process parsing model error: %s' % str(e))
            return

        self.model = YOLO('%s/%s' %
                          (model_ab_path, self.model_info['model_name']))
        self.search_max = rospy.get_param("~search_max", True)
        self.search_centroid = rospy.get_param("~search_centroid", True)
    
        # image frame, lefter-up corner is origin
        self.distance_ratio_x = rospy.get_param("~distance_ratio_x", 0.5)
        self.distance_ratio_y = rospy.get_param("~distance_ratio_y", 0.5)
        self.bridge = CvBridge()
        self.raw_img = None
        self.seg_img = None

        self.compressed_pub_enable = rospy.get_param("~compressed_pub_enable", True)
        self.compressed_raw_pub = rospy.Publisher('compressed_raw_base64', String, queue_size=1, latch=True)
        self.compressed_raw_timer = rospy.Timer(rospy.Duration(1), self.compressed_raw_cb)
        self.compressed_res_pub = rospy.Publisher('compressed_res_base64', String, queue_size=1, latch=True)
        self.compressed_res_timer = rospy.Timer(rospy.Duration(1), self.compressed_res_cb)

        self.img_pub = rospy.Publisher('yolov8_res_img', Image, queue_size=1)
        self.dy_recfg_srv = Server(Yolov8Config, self.dy_recfg_cb)
        rospy.Subscriber('image_publisher/image_raw',
                         Image, self.image_cb, queue_size=1)
        self.last_time = 0
        self.min_rest = rospy.get_param("~min_rest", 0.5)

        self.if_enable = False
        rospy.Subscriber('trig',
                         Header,
                         self.trig_cb,
                         queue_size=10)
        
        self.img_path = relative_to_absolute_path(
            rospy.get_param("~first_image_path", "~/catkin_ws/src/yolov8_result/img/img_first.png"))
        self.model_predict_first()

    def dy_recfg_cb(self, config, level):
        self.distance_ratio_x = config['distance_ratio_x']
        self.distance_ratio_y = config['distance_ratio_y']
        return config

    def trig_cb(self, msg):
        if msg.frame_id == "enable_yolov8" and msg.seq >= 1:
            rospy.loginfo("enable yolov8")
            self.if_enable = True
        elif msg.frame_id == "enable_yolov8" and msg.seq == 0:
            rospy.loginfo("unable yolov8")
            self.if_enable = False

    def model_predict_first(self):
        devicee = torch.device(0) if torch.cuda.is_available() else torch.device('cpu')
        # self.model.predict(source=np.zeros((100,100,3)), save=False, save_txt=False, device=devicee, conf=self.model_info['conf'], iou=self.model_info['iou'], retina_masks=True)
        if os.path.exists(self.img_path):
            img_first = cv2.imread(self.img_path)
            self.model.predict(source=img_first, save=False, save_txt=False, device=devicee, conf=self.model_info['conf'], iou=self.model_info['iou'], retina_masks=True)
            rospy.loginfo("first prediction finished")
    
    def compressed_raw_cb(self, event):
        if (not self.compressed_pub_enable) or (self.raw_img is None):
            return
        # compressed_start = time.time()
        resized_raw_img = cv2.resize(self.raw_img, None, fx=0.2, fy=0.2, interpolation=cv2.INTER_AREA)
        # compressed_end = time.time()
        self.compressed_raw_pub.publish(cv2_to_base64(resized_raw_img))
        # print('compressed raw time: '+str(round((compressed_end-compressed_start)*1000,1))+'ms')
    
    def compressed_res_cb(self, event):
        if (not self.if_enable) or (not self.compressed_pub_enable) or (self.seg_img is None):
            return
        # compressed_start = time.time()
        resized_seg_img = cv2.resize(self.seg_img, None, fx=0.2, fy=0.2, interpolation=cv2.INTER_AREA)
        # compressed_end = time.time()
        self.compressed_res_pub.publish(cv2_to_base64(resized_seg_img))
        # print('compressed res time: '+str(round((compressed_end-compressed_start)*1000,1))+'ms')

    def image_cb(self, image_msg):
        self.raw_img = self.bridge.imgmsg_to_cv2(
            image_msg, desired_encoding='passthrough')

        if (not self.if_enable) or (self.min_rest > 0 and time.time() - self.last_time < self.min_rest):
            return
            
        devicee = torch.device(0) if torch.cuda.is_available() else torch.device('cpu')
        results = self.model.predict(source=self.raw_img, save=False, save_txt=False, device=devicee, conf=self.model_info['conf'], iou=self.model_info['iou'], retina_masks=True)

        for result in results:
            if result.masks is None:
                rospy.loginfo("no result")
                continue
            res_tensor = result.masks.data
            try:
                n = res_tensor.size()[0]
            except:
                n = 0
            if n == 0:
                rospy.loginfo("no result")
                continue
            else:
                self.last_time = time.time()

            max_obj = {'idx': 0, 'area': 0, 'dist_to_c': math.inf}
            for i in range(0, n):
                np_img = res_tensor[i].cpu().numpy()
                # cv2.imwrite("/home/leizeng/catkin_ws/src/my_pkg/yolov8_result/src/res%s.png" % str(i), np.uint8(np_img*255))
                if self.search_max:
                    s = np.sum(np_img)
                    if s > max_obj['area']:
                        max_obj['area'] = s
                        max_obj['idx'] = i
                elif self.search_centroid:
                    cx, cy = self.calculate_centroid(np.uint8(np_img*255))
                    d = np.sqrt((cx - image_msg.width*self.distance_ratio_x)**2 +
                                (cy - image_msg.height*self.distance_ratio_y)**2)
                    if d < max_obj['dist_to_c']:
                        max_obj['dist_to_c'] = d
                        max_obj['idx'] = i
                else:
                    self.img_pub.publish(self.bridge.cv2_to_imgmsg(
                        np.uint8(np_img*255), encoding="mono8"))
            if self.search_max or self.search_centroid:
                np_img = res_tensor[max_obj['idx']].cpu().numpy()
                self.seg_img = np.uint8(np_img*255)
                self.img_pub.publish(self.bridge.cv2_to_imgmsg(
                            self.seg_img, encoding="mono8"))

    def calculate_centroid(self, np_img):
        _, thresh = cv2.threshold(np_img, 127, 255, 0)
        M = cv2.moments(thresh)

        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return (cX, cY)

def relative_to_absolute_path(relative_path):
    print(relative_path)
    if relative_path[0] == '~':
        return '/home/' + getpass.getuser() + relative_path[1:]
    else:
        return relative_path

def main():
    rospy.init_node('yolov8_result_node')
    Yolov8Process()
    try:
        rospy.spin()
    except:
        print("Shutting down...")


if __name__ == '__main__':
    main()
