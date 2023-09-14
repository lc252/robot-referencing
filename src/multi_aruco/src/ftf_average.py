#!/usr/bin/env python3

import rospy
import numpy as np
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from geometry_msgs.msg import Vector3, Quaternion, PoseWithCovarianceStamped, TransformStamped
from scipy.spatial.transform import Rotation as R
import tf2_ros as tf2



class ftf_average():
    def __init__(self):
        self.ftf_sub = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.average_transforms)
        self.pose_pub = rospy.Publisher(f"/pose/filtered", PoseWithCovarianceStamped, queue_size=1)
        self.buffer = tf2.Buffer()
        self.listener = tf2.TransformListener(self.buffer)
        # wait for buffer
        rospy.sleep(1)

    def average_transforms(self, ftf_arr : FiducialTransformArray):
        # gets each fiducial transform and transforms it to the base frame
        # uses the object error as the covariance for each meseaurement
        pwcs = PoseWithCovarianceStamped()
        pwcs.header.stamp = rospy.Time.now() # ftf_arr.header.stamp
        pwcs.header.frame_id = "base_link"

        t = np.zeros((len(ftf_arr.transforms), 3))
        q = np.zeros((len(ftf_arr.transforms), 4))
        rpy = np.zeros((len(ftf_arr.transforms), 3))

        idx = 0

        ftf : FiducialTransform
        for ftf in ftf_arr.transforms:
            # for each aruco, find the transform from the base of the robot to the camera. The chain is base_link->aruco->optical_frame->camera
            # Initially these transforms are unlinked. The aruco position is related to the base frame by a known, static transform.
            # camera_link to optical_frame is static and known, and optical_frame to aruco is measured and varies.
            # By comparing the measured position of the aruco relative to the camera to its known position relative to the base, we can find the transform from the camera to the base, and the inverse.

            # get the known base -> aruco transform: lookup_transform(target_frame, source_frame, time)
            ar_base_TF:TransformStamped = self.buffer.lookup_transform(f"aruco{ftf.fiducial_id}", "base_link", rospy.Time())
            ar_base_M = self.tf_matrix(ar_base_TF.transform.translation, ar_base_TF.transform.rotation)

            # get the measured: optical_frame->aruco transform
            opt_ar_M = self.tf_matrix(ftf.transform.translation, ftf.transform.rotation)

            # get camera_link->optical_frame
            cam_opt_TF:TransformStamped = self.buffer.lookup_transform("camera_link", "camera_color_optical_frame", rospy.Time())
            cam_opt_M = self.tf_matrix(cam_opt_TF.transform.translation, cam_opt_TF.transform.rotation)
            
            # camera_link->aruco transform matrix by multiplying camera_link->optical_frame by optical_frame->aruco
            cam_ar_M = np.matmul(cam_opt_M, opt_ar_M)
            # get the camera->base transform by multiplying the cam->ar by ar->base
            cam_base_M = np.matmul(cam_ar_M, ar_base_M)
            # invert the transform to get base -> camera
            base_cam_M = np.linalg.inv(cam_base_M)

            # store translation and rotation
            t[idx] = base_cam_M[0:3, 3]
            q[idx] = R.from_matrix(base_cam_M[0:3,0:3]).as_quat()
            rpy[idx] = R.from_matrix(base_cam_M[0:3,0:3]).as_euler("xyz")

            idx += 1
            
        # average the translation and rotation
        t_mean = np.mean(t, axis=0)
        q_mean = np.mean(q, axis=0)
        t_std = np.std(t, axis=0)
        q_std = np.std(q, axis=0)
        rpy_std = np.std(rpy, axis=0)
        # remove values outside 1 stds
        t = t[np.all(np.abs(t - t_mean) < 1 * t_std, axis=1)]
        q = q[np.all(np.abs(q - q_mean) < 1 * q_std, axis=1)]
        # recompute mean
        t_mean = np.mean(t, axis=0)
        q_mean = np.mean(q, axis=0)
        # convert q to euler, roll, pitch, yaw
        rpy = R.from_quat(q_mean).as_euler("xyz")

        # fill covariance matrix
        cov = np.array([[t_std[0], 0, 0, 0, 0, 0],
                       [0, t_std[1], 0, 0, 0, 0],
                       [0, 0, t_std[2], 0, 0, 0],
                       [0, 0, 0, rpy_std[0], 0, 0],
                       [0, 0, 0, 0, rpy_std[1], 0],
                       [0, 0, 0, 0, 0, rpy_std[2]]])

        # populate the message fields
        pwcs.pose.pose.position = Vector3(*t_mean)
        pwcs.pose.pose.orientation = Quaternion(*q_mean)
        pwcs.pose.covariance = cov.flatten()
             
        # publish
        self.pose_pub.publish(pwcs)

    def transform_to_base_frame(self, ftf_arr : FiducialTransformArray):
        # gets each fiducial transform and transforms it to the base frame
        # uses the object error as the covariance for each meseaurement
        pwcs = PoseWithCovarianceStamped()
        pwcs.header.stamp = rospy.Time.now() # ftf_arr.header.stamp
        pwcs.header.frame_id = "base_link"

        ftf : FiducialTransform
        for ftf in ftf_arr.transforms:
            # get the known base -> aruco transform
            ar_base_TF : TransformStamped
            ar_base_TF = self.buffer.lookup_transform(f"aruco{ftf.fiducial_id}", "base_link", rospy.Time())
            # extract the translation and rotation
            ar_base_t = ar_base_TF.transform.translation
            ar_base_q = ar_base_TF.transform.rotation
            ar_base_r = R.from_quat([ar_base_q.x, ar_base_q.y, ar_base_q.z, ar_base_q.w])
            # build the transform matrix
            ar_base_M = np.identity(4)
            ar_base_M[0:3,0:3] = ar_base_r.as_matrix()
            ar_base_M[0:3, 3] = np.array([ar_base_t.x, ar_base_t.y, ar_base_t.z])

            # get the camera optical -> aruco transform
            opt_ar_t = ftf.transform.translation
            opt_ar_q = ftf.transform.rotation
            opt_ar_r = R.from_quat([opt_ar_q.x, opt_ar_q.y, opt_ar_q.z, opt_ar_q.w])
            # build the transform matrix
            opt_ar_M = np.identity(4)
            opt_ar_M[0:3,0:3] = opt_ar_r.as_matrix()
            opt_ar_M[0:3, 3] = np.array([opt_ar_t.x, opt_ar_t.y, opt_ar_t.z])

            # get cam -> opt
            cam_opt_TF : TransformStamped
            cam_opt_TF = self.buffer.lookup_transform("camera_link", "camera_color_optical_frame", rospy.Time())
            # extract the translation and rotation
            cam_opt_t = cam_opt_TF.transform.translation
            cam_opt_q = cam_opt_TF.transform.rotation
            cam_opt_r = R.from_quat([cam_opt_q.x, cam_opt_q.y, cam_opt_q.z, cam_opt_q.w])
            # build the transform matrix
            cam_opt_M = np.identity(4)
            cam_opt_M[0:3,0:3] = cam_opt_r.as_matrix()
            cam_opt_M[0:3, 3] = np.array([cam_opt_t.x, cam_opt_t.y, cam_opt_t.z])   
            cam_ar_M = np.matmul(cam_opt_M, opt_ar_M)
            

            # get the camera -> base transform
            cam_base_M = np.matmul(cam_ar_M, ar_base_M)
            # invert the transform to get base -> camera
            base_cam_M = np.linalg.inv(cam_base_M)
            # extract the rotation matrix
            base_cam_r = R.from_matrix(base_cam_M[0:3,0:3])    

            # populate the diagonal of the covariance matrix with the object error (reprojection error in m)
            cov = ftf.object_error * np.identity(6)

            # populate the message fields
            q = base_cam_r.as_quat()
            pwcs.pose.pose.position = Vector3(*base_cam_M[0:3, 3])
            pwcs.pose.pose.orientation = Quaternion(*q)
            pwcs.pose.covariance = cov.flatten()

            # publish
            self.pose_pub.publish(pwcs)

    def tf_matrix(self, translation : Vector3, rotation : Quaternion):
        # build a transform matrix from a translation vector and a rotation quaternion
        r = R.from_quat([rotation.x, rotation.y, rotation.z, rotation.w])
        m = np.identity(4)
        m[0:3,0:3] = r.as_matrix()
        m[0:3, 3] = np.array([translation.x, translation.y, translation.z])
        return m



if __name__ == "__main__":
    rospy.init_node("converter")
    n = ftf_average()
    while not rospy.is_shutdown():
        rospy.spin()