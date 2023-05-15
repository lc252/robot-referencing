#!/usr/bin/env python3

import rospy
from ultralytics import YOLO
import numpy as np
import cv2
from sensor_msgs.msg import Image



class Detector:
    def __init__(self, render=False):
        # load inference model
        self.model = YOLO("yolov8n-seg.pt")
        self.model.to("cuda")
        # image subscriber
        self.sub = rospy.Subscriber('camera/color/image_raw', Image, self.new_image_cb)
        self.img = np.zeros(shape=(480,640,3))
        # timer callback to perform inference. on a separate thread for efficiency
        self.timer = rospy.timer.Timer(rospy.Duration(1/30), self.inference_cb)
        self.render = render

    def new_image_cb(self, img_msg: Image):
        self.img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)

    def inference_cb(self, _):
        # do detection
        results = self.model(self.img)
        # render result in cv     
        if self.render:
            self.render_cb(results)
    
    def render_cb(self, results):
        cv2.imshow('Stream', results[0].plot())
        cv2.waitKey(1)



if __name__ == '__main__':
    rospy.init_node('yolo_inference_node', anonymous=True)
    display = Detector(render=True)
    rospy.spin()