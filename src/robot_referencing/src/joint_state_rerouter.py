#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock



class joint_rerouter():
    def __init__(self):
        self.joint_sub = rospy.Subscriber("/joint_states", JointState, self.clock_cb)
        self.joint2_pub = rospy.Publisher("/joint_states2", JointState, queue_size=1)

    def clock_cb(self, msg : JointState):
        # update stamp
        msg.header.stamp = rospy.Time.now()
        self.joint2_pub.publish(msg)



if __name__ == "__main__":
    rospy.init_node("clock_rerouter", log_level=rospy.ERROR)
    jr = joint_rerouter()
    rospy.spin()