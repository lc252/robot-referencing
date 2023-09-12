#!/usr/bin/env python3

import rospy
import numpy as np
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from geometry_msgs.msg import Vector3, Quaternion, PoseWithCovarianceStamped, TransformStamped
from scipy.spatial.transform import Rotation as R
import tf2_ros as tf2



class ftf_average():
    def __init__(self):
        self.ftf_sub = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.transform_to_base_frame)
        self.pose_pub = rospy.Publisher(f"/pose/filtered", PoseWithCovarianceStamped, queue_size=1)
        self.buffer = tf2.Buffer()
        self.listener = tf2.TransformListener(self.buffer)
        # wait for buffer
        rospy.sleep(1)

    def average_transforms(self, ftf_arr : FiducialTransformArray):
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
            # extract the euler angles

            # this and the line below showed that this method did not work
            base_cam_eul = base_cam_r.as_euler("xyz")  
            # rospy.logerr(f"\n{np.around(base_cam_M[0:3,0:3])}\n{base_cam_r.as_quat()}\n{np.around(base_cam_eul, 2)}")
            
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



if __name__ == "__main__":
    rospy.init_node("converter")
    n = ftf_average()
    while not rospy.is_shutdown():
        rospy.spin()