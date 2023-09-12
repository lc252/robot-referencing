#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock



class clock_rerouter():
    def __init__(self):
        self.clock_sub = rospy.Subscriber("/joint_states", JointState, self.clock_cb)
        self.clock_pub = rospy.Publisher("/clock", Clock, queue_size=1)

    def clock_cb(self, msg : JointState):
        time = Clock()
        time.clock = msg.header.stamp
        self.clock_pub.publish(time)



if __name__ == "__main__":
    rospy.init_node("clock_rerouter", log_level=rospy.ERROR)
    cr = clock_rerouter()
    rospy.spin()