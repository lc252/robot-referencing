#!/usr/bin/env python3

import rospy
from ultralytics import YOLO
import numpy as np
import cv2
from sensor_msgs.msg import Image, PointCloud2
import message_filters
from yolo.msg import Mask, MaskArray



class Detector:
    def __init__(self):
        # load params
        self.params = rospy.get_param("yolo/")

        # load inference model
        self.model = YOLO(self.params["model"])
        if self.params["cuda"]:
            self.model.to("cuda")
        
        # synchronised image / cloud subscribers
        # image subscriber
        self.img_sub = message_filters.Subscriber("camera/color/image_raw", Image)
        self.img = np.zeros(shape=(480,640,3))
        # pointcloud subscriber
        self.pc_sub = message_filters.Subscriber("camera/depth_registered/points", PointCloud2)
        self.cloud = PointCloud2()
        self.sync_sub = message_filters.TimeSynchronizer([self.img_sub, self.pc_sub], queue_size=1)
        self.sync_sub.registerCallback(self.sync_cb)

        # Detection Mask Publisher
        self.mask_pub = rospy.Publisher("yolo/detections", MaskArray, queue_size=1)

        # timer callback to perform inference. on a separate thread for efficiency
        self.timer = rospy.timer.Timer(rospy.Duration(1/30), self.inference_cb)

    def sync_cb(self, img_msg, pc2_msg):
        self.img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)
        self.cloud = pc2_msg

    def inference_cb(self, timer):
        # do detection
        results = self.model.predict(self.img, show=self.params["render"])
        masks = results[0].masks if results[0].masks != None else []
        boxes = results[0].boxes if results[0].boxes != None else []
        
        # detections message
        msg = MaskArray()

        # iterate detections
        for box, mask in zip(boxes, masks):
            det = Mask(cls=int(box.cls), 
                       conf=float(box.conf), 
                       width=int(np.shape(self.img)[0]), 
                       height=int(np.shape(self.img)[1]), 
                       poly=tuple(np.ndarray.flatten(mask.xy[0]).astype(int)))
            msg.masks.append(det)

        self.mask_pub.publish(msg)



if __name__ == '__main__':
    rospy.init_node('yolo_inference_node', anonymous=True, log_level=rospy.DEBUG)
    display = Detector()
    rospy.spin() 