#!/usr/bin/env python3

import rospy
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


class plotter():
    def __init__(self):
        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.callback)
        self.df = pd.DataFrame(columns=["x", "y", "z", "qx", "qy", "qz", "qw"])

    def callback(self, msg : TFMessage):
        tf : TransformStamped
        for tf in msg.transforms:
            if tf.header.frame_id == "world" and tf.child_frame_id == "aligned":
                rospy.loginfo("Adding TF")
                # self.df = self.df.append({"x": tf.transform.translation.x, "y": tf.transform.translation.y, "z": tf.transform.translation.z, "qx": tf.transform.rotation.x, "qy": tf.transform.rotation.y, "qz": tf.transform.rotation.z, "qw": tf.transform.rotation.w}, ignore_index=True)
                self.df = pd.concat([self.df, pd.DataFrame({"x": tf.transform.translation.x, "y": tf.transform.translation.y, "z": tf.transform.translation.z, "qx": tf.transform.rotation.x, "qy": tf.transform.rotation.y, "qz": tf.transform.rotation.z, "qw": tf.transform.rotation.w}, index=[0])])
                self.df.to_csv("/home/fif/lc252/measurements/static_camera_frame_moving_robot.csv")



if __name__ == "__main__":
    rospy.init_node("plotter")
    p = plotter()
    rospy.spin()