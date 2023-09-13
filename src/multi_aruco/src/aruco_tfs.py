#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped, Transform
from std_msgs import msg
import tf2_ros as tf2



class aruco_pos_sim():
    def __init__(self):
        self.broadcaster = tf2.StaticTransformBroadcaster()
        self.timer = rospy.Timer(rospy.Duration(0.01), self.publish_tfs)

        # self.transforms = {
        #     "aruco0"  : TransformStamped(msg.Header(0, rospy.Time.now(), "base_link"),  "aruco0", Transform(Vector3(0.113, 0, 0.120), Quaternion(0.5, 0.5, 0.5, 0.5))),
        #     "aruco1"  : TransformStamped(msg.Header(0, rospy.Time.now(), "base_link"),  "aruco1", Transform(Vector3(0, 0.113, 0.120), Quaternion(0, 0.7071, 0.7071, 0))),
        #     "aruco2"  : TransformStamped(msg.Header(0, rospy.Time.now(), "base_link"),  "aruco2", Transform(Vector3(0, -0.113, 0.120), Quaternion(0, 0, 0, 1))),
        #     "aruco3"  : TransformStamped(msg.Header(0, rospy.Time.now(), "link_1"),     "aruco3", Transform(Vector3(0, -0.130, 0), Quaternion(0.5, 0.5, 0.5, 0.5))),
        #     "aruco4"  : TransformStamped(msg.Header(0, rospy.Time.now(), "link_1"),     "aruco4", Transform(Vector3(0, 0.130, 0), Quaternion(0, 0.7071, 0.7071, 0))),
        #     "aruco5"  : TransformStamped(msg.Header(0, rospy.Time.now(), "link_2"),     "aruco5", Transform(Vector3(0.021, 0, 0.130), Quaternion(0.5, 0.5, 0.5, 0.5))),
        #     "aruco6"  : TransformStamped(msg.Header(0, rospy.Time.now(), "link_2"),     "aruco6", Transform(Vector3(0, -0.118, 0.350), Quaternion(0.5, 0.5, 0.5, 0.5))),
        #     "aruco7"  : TransformStamped(msg.Header(0, rospy.Time.now(), "link_2"),     "aruco7", Transform(Vector3(0, 0.118, 0.350), Quaternion(0, 0.7071, 0.7071, 0))),
        #     "aruco8"  : TransformStamped(msg.Header(0, rospy.Time.now(), "link_3"),     "aruco8", Transform(Vector3(0.267, 0.057, 0), Quaternion(0, 0.7071, 0.7071, 0))),
        #     "aruco9"  : TransformStamped(msg.Header(0, rospy.Time.now(), "link_3"),     "aruco9", Transform(Vector3(0.130, -0.059, 0.042), Quaternion(0, 0.7071, 0.7071, 0))),
        #     "aruco10" : TransformStamped(msg.Header(0, rospy.Time.now(), "link_3"),    "aruco10", Transform(Vector3(0.130, 0.059, 0.042), Quaternion(0, 0.7071, 0.7071, 0))),
        #     "aruco11" : TransformStamped(msg.Header(0, rospy.Time.now(), "link_4"),    "aruco11", Transform(Vector3(0.267, -0.057, 0), Quaternion(0, 0, 0, 1)))
        # }

        # second test
        self.transforms = {
            "aruco0"  : TransformStamped(msg.Header(0, rospy.Time.now(), "base_link"),  "aruco0", Transform(Vector3(0.113, 0, 0.120), Quaternion(0.5, 0.5, 0.5, 0.5))),
            "aruco1"  : TransformStamped(msg.Header(0, rospy.Time.now(), "base_link"),  "aruco1", Transform(Vector3(-0.006, 0.113, 0.126), Quaternion(-0.479, -0.506, -0.496, 0.536))),
            "aruco2"  : TransformStamped(msg.Header(0, rospy.Time.now(), "base_link"),  "aruco2", Transform(Vector3(0, -0.113, 0.120), Quaternion(0, 0, 0, 1))),
            "aruco3"  : TransformStamped(msg.Header(0, rospy.Time.now(), "link_1"),     "aruco3", Transform(Vector3(0, -0.130, 0), Quaternion(0.5, 0.5, 0.5, 0.5))),
            "aruco4"  : TransformStamped(msg.Header(0, rospy.Time.now(), "link_1"),     "aruco4", Transform(Vector3(0, 0.130, 0), Quaternion(0, 0.7071, 0.7071, 0))),
            "aruco5"  : TransformStamped(msg.Header(0, rospy.Time.now(), "link_2"),     "aruco5", Transform(Vector3(0.021, 0, 0.130), Quaternion(0.5, 0.5, 0.5, 0.5))),
            "aruco6"  : TransformStamped(msg.Header(0, rospy.Time.now(), "link_2"),     "aruco6", Transform(Vector3(0, -0.118, 0.350), Quaternion(0.5, 0.5, 0.5, 0.5))),
            "aruco7"  : TransformStamped(msg.Header(0, rospy.Time.now(), "link_2"),     "aruco7", Transform(Vector3(0, 0.118, 0.350), Quaternion(0, 0.7071, 0.7071, 0))),
            "aruco8"  : TransformStamped(msg.Header(0, rospy.Time.now(), "link_3"),     "aruco8", Transform(Vector3(0.267, 0.057, 0), Quaternion(0, 0.7071, 0.7071, 0))),
            "aruco9"  : TransformStamped(msg.Header(0, rospy.Time.now(), "link_3"),     "aruco9", Transform(Vector3(0.130, -0.059, 0.042), Quaternion(0, 0.7071, 0.7071, 0))),
            "aruco10" : TransformStamped(msg.Header(0, rospy.Time.now(), "link_3"),    "aruco10", Transform(Vector3(0.130, 0.059, 0.042), Quaternion(0, 0.7071, 0.7071, 0))),
            "aruco11" : TransformStamped(msg.Header(0, rospy.Time.now(), "link_4"),    "aruco11", Transform(Vector3(0.267, -0.057, 0), Quaternion(0, 0, 0, 1)))
        }
    
    def publish_tfs(self, timer_event):
        for aruco in self.transforms:
            tf = self.transforms[aruco]
            tf.header.stamp = rospy.Time.now()
            self.broadcaster.sendTransform(tf)

        # for offline
        # self.broadcaster.sendTransform(TransformStamped(msg.Header(0, rospy.Time.now(), "world"), "base_link", Transform(Vector3(0, 0, 0), Quaternion(0, 0, 0, 1))))
        #self.broadcaster.sendTransform(TransformStamped(msg.Header(0, rospy.Time.now(), "camera_link"), "camera_color_optical_frame", Transform(Vector3(0, 0.015, 0), Quaternion(0.5, 0.5, 0.5, -0.5))))



if __name__ == "__main__":
    rospy.init_node("aruco_pos_sim")
    aps = aruco_pos_sim()
    rospy.spin()