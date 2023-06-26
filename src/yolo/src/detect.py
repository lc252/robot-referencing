#!/usr/bin/env python3

import rospy
from ultralytics import YOLO
import numpy as np
import cv2
from sensor_msgs.msg import Image, PointCloud2



class Detector:
    def __init__(self):
        # load inference model
        self.model = YOLO("models/yolov8n-seg.pt")
        self.params = rospy.get_param("yolo/")
        if self.params["cuda"]:
            self.model.to("cuda")
        # image subscriber
        self.img_sub = rospy.Subscriber("camera/color/image_raw", Image, self.image_cb)
        self.img = np.zeros(shape=(480,640,3))
        # pointcloud subscriber
        self.pc_sub = rospy.Subscriber("camera/")
        # timer callback to perform inference. on a separate thread for efficiency
        self.timer = rospy.timer.Timer(rospy.Duration(1/30), self.inference_cb)

    def image_cb(self, img_msg: Image):
        self.img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)

    def pc_callback(self, pc2_msg: PointCloud2):
        pass

    def inference_cb(self, timer):
        # do detection
        results = self.model.predict(self.img, show=self.params["render"])



if __name__ == '__main__':
    rospy.init_node('yolo_inference_node', anonymous=True)
    display = Detector()
    rospy.spin()