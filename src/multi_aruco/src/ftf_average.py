#!/usr/bin/env python3

import rospy
import numpy as np
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from geometry_msgs.msg import Vector3, Quaternion, PoseWithCovarianceStamped, TransformStamped
from scipy.spatial.transform import Rotation as R
import tf2_ros as tf2



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
            # get the known base -> aruco transform
            ar_base_TF : TransformStamped
            ar_base_TF = self.buffer.lookup_transform(f"aruco_{ftf.fiducial_id}_known", "base_link", rospy.Time())
            # extract the translation and rotation
            ar_base_t = ar_base_TF.transform.translation
            ar_base_q = ar_base_TF.transform.rotation
            ar_base_r = R.from_quat([ar_base_q.x, ar_base_q.y, ar_base_q.z, ar_base_q.w])
            # build the transform matrix
            ar_base_M = np.identity(4)
            ar_base_M[0:3,0:3] = ar_base_r.as_matrix()
            ar_base_M[0:3, 3] = np.array([ar_base_t.x, ar_base_t.y, ar_base_t.z])

            # get the camera -> aruco transform
            cam_ar_t = ftf.transform.translation
            cam_ar_q = ftf.transform.rotation
            cam_ar_r = R.from_quat([cam_ar_q.x, cam_ar_q.y, cam_ar_q.z, cam_ar_q.w])
            # build the transform matrix
            cam_ar_M = np.identity(4)
            cam_ar_M[0:3,0:3] = cam_ar_r.as_matrix()
            cam_ar_M[0:3, 3] = np.array([cam_ar_t.x, cam_ar_t.y, cam_ar_t.z])

            # get the camera -> base transform
            cam_base_M = np.matmul(cam_ar_M, ar_base_M)
            # invert the transform to get base -> camera
            base_cam_M = np.linalg.inv(cam_base_M)
            # extract the rotation matrix
            base_cam_r = R.from_matrix(base_cam_M[0:3,0:3])
            # extract the euler angles
            base_cam_eul = base_cam_r.as_euler("xyz")  
            
            # append rotation and translation to lists for averaging
            roll.append(base_cam_eul[0])
            pitch.append(base_cam_eul[1])
            yaw.append(base_cam_eul[2])
            x.append(base_cam_M[0,3])
            y.append(base_cam_M[1,3])
            z.append(base_cam_M[2,3])

        # calculate the covariance matrix
        cov = np.array([[np.var(x), 0, 0, 0, 0, 0],
                        [0, np.var(y), 0, 0, 0, 0],
                        [0, 0, np.var(z), 0, 0, 0],
                        [0, 0, 0, np.var(roll), 0, 0],
                        [0, 0, 0, 0, np.var(pitch), 0],
                        [0, 0, 0, 0, 0, np.var(yaw)]])

        # calculate the average rotation and translation
        q = R.from_euler("xyz", [np.mean(roll), np.mean(pitch), np.mean(yaw)]).as_quat()
        pwcs.pose.pose.position = Vector3(np.mean(x), np.mean(y), np.mean(z))
        pwcs.pose.pose.orientation = Quaternion(*q)
        # pwcs.pose.covariance = cov.flatten()

        # publish
        self.pose_pub.publish(pwcs)



if __name__ == "__main__":
    rospy.init_node("converter")
    n = ftf_average()
    while not rospy.is_shutdown():
        rospy.spin()