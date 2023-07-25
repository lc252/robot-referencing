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
        
        x = np.mean(x)
        x_cov = np.std(x_cov)
        y = np.mean(y)
        y_cov = np.std(y_cov)
        z = np.mean(z)
        z_cov = np.std(z_cov)
        roll = np.mean(roll)
        roll_cov = np.std(roll_cov)
        pitch = np.mean(pitch)
        pitch_cov = np.std(pitch_cov)
        yaw = np.mean(yaw)
        yaw_cov = np.std(yaw_cov)

        cov = np.array([[x_cov, 0, 0, 0, 0, 0],
                        [0, y_cov, 0, 0, 0, 0],
                        [0, 0, z_cov, 0, 0, 0],
                        [0, 0, 0, roll_cov, 0, 0],
                        [0, 0, 0, 0, pitch_cov, 0],
                        [0, 0, 0, 0, 0, yaw_cov]])
        cov = cov**2

        q = transformations.quaternion_from_euler(roll, pitch, yaw)
        pwcs.pose.pose.position = Vector3(x, y, z)
        pwcs.pose.pose.orientation = Quaternion(*q)
        pwcs.pose.covariance = cov.flatten()

        # publish
        self.pose_pub.publish(pwcs)



if __name__ == "__main__":
    rospy.init_node("converter")
    n = ftf_average()
    while not rospy.is_shutdown():
        rospy.spin()