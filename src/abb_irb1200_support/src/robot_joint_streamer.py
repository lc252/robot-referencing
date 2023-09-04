#!/usr/bin/env python3

import rospy
from RWS import RWS
from sensor_msgs.msg import JointState
import numpy as np



class RobotJointStreamer(RWS):
    # class implements RWS wrapper to connect to robot and stream joint values to ROS
    def __init__(self, base_url=None):
        if base_url == None:
            self.use_real_robot = False
        else:
            super().__init__(base_url)
            self.use_real_robot = True

        self.timer = rospy.Timer(rospy.Duration(1/30), self.stream_joint_values)
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    
    def stream_joint_values(self, timer_event):
        # stream joint values to ROS
        if self.use_real_robot and self.is_connected():
            joint_values = self.request_joint_values()
        else:
            joint_values = self.all_zero_joint_values()
        
        self.publish_joint_values(joint_values)
    
    def request_joint_values(self):
        joints = JointState()
        joints.header.stamp = rospy.Time.now()
        joints.header.frame_id = 'base_link'
        joints.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        joints.position = self.get_joints()
        # convert to radians
        joints.position = [j*(np.pi/180) for j in joints.position]
        return joints

    def all_zero_joint_values(self):
        # stub to publish all zero joint values
        joints = JointState()
        joints.header.stamp = rospy.Time.now()
        joints.header.frame_id = 'base_link'
        joints.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        joints.position = [0, 0, 0, 0, 0, 0]
        return joints
    
    def publish_joint_values(self, joint_values):
        self.joint_pub.publish(joint_values)



if __name__ == '__main__':
    rospy.init_node('robot_joint_streamer_node')
    robot_connection_node = RobotJointStreamer("http://192.168.125.1")
    rospy.spin()