#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion
from tf import TransformBroadcaster


class world_to_unity_node:
    def __init__(self):
        self.pose_sub = rospy.Subscriber("unity_objects/alignment_guess", PoseStamped, self.pose_cb)
        self.br = TransformBroadcaster()

    def pose_cb(self, pose : PoseStamped):
        pos = (-pose.pose.position.x, -pose.pose.position.y, -pose.pose.position.z)
        rot = (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, -pose.pose.orientation.w)
        self.br.sendTransform(pos, rot, rospy.Time.now(), "unity", "base_link")


if __name__ == '__main__':
    rospy.init_node('yolo_inference_node', anonymous=True, log_level=rospy.DEBUG)
    wun = world_to_unity_node()
    rospy.spin() 
    