#!/usr/bin/env python3


import rospy
from sensor_msgs.msg import PointCloud2
import numpy as np



class AxisInverter():
    """
    This class is used to invert an axis of pointcloud2 data to change it from the Unity LHCS to the ROS RHCS.
    This script is used for testing purposes only. The HL2 Research Mode Plugin should be used to invert the axis.
    """
    def __init__(self):
        self.sub = rospy.Subscriber("/camera/ahat", PointCloud2, self.invert_axis_callback)
        self.pub = rospy.Publisher("/camera/inverted", PointCloud2, queue_size=1)

    def invert_axis_callback(self, cloud : PointCloud2):
        # get the pointcloud2 data
        data = PointCloud2.read_points(cloud, field_names=("x","y","z"), skip_nans=True)
        # get the point step size
        point_step = cloud.point_step
        # get the x index
        x_index = cloud.fields[0].offset

        # iterate the cloud data
        for i in range(0, len(data), point_step):
            # invert the x axis
            data[i+x_index] = -data[i+x_index]

        # fill the old cloud with the new data
        cloud.data = data.astype(np.uint8).tobytes()

        # one point has 3 float32, therefore 12 bytes req.
        cloud.point_step = len(cloud.fields) * np.dtype(np.float32).itemsize
        
        # num data points (1000) * num fields per point (3)
        cloud.row_step = cloud.width * point_step

        # publish
        self.pub.publish(cloud)



if __name__ == "__main__":
    rospy.init_node("invert_axis_test", log_level=rospy.ERROR)
    ai = AxisInverter()
    rospy.spin()