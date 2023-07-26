#!/usr/bin/env python3

import rospy
import numpy as np
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from geometry_msgs.msg import Vector3, Quaternion, PoseWithCovarianceStamped
from tf import transformations



class ftf_average():
    def __init__(self):
        self.ftf_sub = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.conversion_cb)
        self.pose_pub = rospy.Publisher(f"/pose/filtered", PoseWithCovarianceStamped, queue_size=1)

    def conversion_cb(self, ftf_arr : FiducialTransformArray):
        pwcs = PoseWithCovarianceStamped()
        pwcs.header.stamp = ftf_arr.header.stamp
        pwcs.header.frame_id = "base_link"
        
        x = []
        y = []
        z = []
        roll = []
        pitch = []
        yaw = []

        ftf : FiducialTransform
        for ftf in ftf_arr.transforms:
            euler = transformations.euler_from_quaternion([ftf.transform.rotation.x, ftf.transform.rotation.y, ftf.transform.rotation.z, ftf.transform.rotation.w,])
            roll.append(euler[0])
            pitch.append(euler[1])
            yaw.append(euler[2])

            x.append(ftf.transform.translation.x)
            y.append(ftf.transform.translation.y)
            z.append(ftf.transform.translation.z)
        
        cov = np.array([[np.std(x), 0, 0, 0, 0, 0],
                        [0, np.std(y), 0, 0, 0, 0],
                        [0, 0, np.std(z), 0, 0, 0],
                        [0, 0, 0, np.std(roll), 0, 0],
                        [0, 0, 0, 0, np.std(pitch), 0],
                        [0, 0, 0, 0, 0, np.std(yaw)]])
        cov = cov**2

        q = transformations.quaternion_from_euler(np.mean(roll), np.mean(pitch), np.mean(yaw))
        pwcs.pose.pose.position = Vector3(np.mean(x), np.mean(y), np.mean(z))
        pwcs.pose.pose.orientation = Quaternion(*q)
        pwcs.pose.covariance = cov.flatten()

        # publish
        self.pose_pub.publish(pwcs)



if __name__ == "__main__":
    rospy.init_node("converter")
    n = ftf_average()
    while not rospy.is_shutdown():
        rospy.spin()