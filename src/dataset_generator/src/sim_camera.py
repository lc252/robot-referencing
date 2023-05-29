#!/usr/bin/env python3

import rospy
from tf import TransformBroadcaster, transformations
from std_srvs.srv import SetBool, SetBoolResponse
from sensor_msgs.msg import Image as ROS_Image
from geometry_msgs.msg import Quaternion
from PIL import Image as PIL_Image
import matplotlib.pyplot as plt
import cv2
from cv_bridge import CvBridge
import numpy as np
import time
import datetime



class camera_node():
    def __init__(self):
        self.image_sub = rospy.Subscriber("camera1/camera1/image", ROS_Image, self.image_cb)
        self.bridge = CvBridge()
        self.im_arr = None
        
        self.br = TransformBroadcaster()

        try:
            rospy.wait_for_service("target_planner/plan_request", 5)
        except Exception as e:
            rospy.logerr("%s", e)
        self.plan_srv = rospy.ServiceProxy("target_planner/plan_request", SetBool)

        split = {
            "train" : 100,
            "test" : 14,
            "val" : 14
        }
        for dset in split.keys():
            for i in range(split[dset]):
                self.capture(dset, i)

    def capture(self, dset, i):
        success = False
        while success == False:
            success = self.call_planner_service()

        roll = -np.pi/2 
        for theta in range(0,359,45):
            # randomise capture location
            r = np.random.rand() + 0.5  # random float from 0.5 - 1.5
            z = np.random.rand()*0.5 + 0.25  # random float from 0.25 - 0.75

            yaw = theta*(np.pi/180)
            x = r*np.cos(yaw)
            y = r*np.sin(yaw)
            q = transformations.quaternion_from_euler(roll, 0, yaw + np.pi/2)
            self.br.sendTransform((x,y,z), (q[0],q[1],q[2],q[3]), rospy.Time.now(), "camera1", "world")
            time.sleep(1.1)
            tag = datetime.datetime.now().strftime("%d%M%S")
            rospy.loginfo("capturing image_%s", tag)
            # plt.imsave(f"/home/fif/lc252/datasets/{dset}/images/image_{tag}.png", self.im_arr)

    def call_planner_service(self):
        resp = self.plan_srv(True)
        return resp.success

    def image_cb(self, image):
        self.im_arr = self.bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
        self.im_arr = np.asarray(self.im_arr)
        # bgr2rgb
        # self.im_arr = self.im_arr[...,::-1].copy()
        # cv2.imshow("image", self.im_arr)
        # cv2.waitKey(80)



if __name__ == "__main__":
    rospy.init_node("camera_transformer")
    nh = camera_node()

    while not rospy.is_shutdown():
        rospy.spin()