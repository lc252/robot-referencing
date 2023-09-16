// ROS 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>



// Types
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;



class ahat_aggregate_node
{
public:
    ahat_aggregate_node():
        nh{},
        cloud_sub(nh.subscribe("cloud_in", 1, &ahat_aggregate_node::cloud_cb, this)),
        cloud_pub(nh.advertise<sensor_msgs::PointCloud2>("cloud_out", 1))
    {
        ros::param::get("/registration_node/preprocess/downsample_leaf_size", downsample_leaf_size);
        cloud = PointCloudXYZ::Ptr(new PointCloudXYZ);
    };
    
    ~ahat_aggregate_node()
    {
        ;
    }

private:
    // ROS
    ros::NodeHandle nh;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub;

    // clouds
    PointCloudXYZ::Ptr cloud;

    // param
    float downsample_leaf_size;

    void cloud_cb(sensor_msgs::PointCloud2 cloud_in)
    {
        PointCloudXYZ::Ptr new_cloud(new PointCloudXYZ);

        // load cloud
        ROS_INFO("Loading Cloud");
        pcl::fromROSMsg(cloud_in, *new_cloud);;

        // add and downsample
        *cloud += *new_cloud;
        downsample(cloud, downsample_leaf_size);

        // publish
        sensor_msgs::PointCloud2 cloud_out;
        pcl::toROSMsg(*cloud, cloud_out);
        cloud_out.header.frame_id = cloud_in.header.frame_id;
        cloud_pub.publish(cloud_out);
    }

    void downsample(PointCloudXYZ::Ptr &cloud, float leaf_size)
    {
        pcl::VoxelGrid<pcl::PointXYZ> grid;
        grid.setLeafSize(leaf_size, leaf_size, leaf_size);
        grid.setInputCloud(cloud);
        grid.filter(*cloud);
    }

};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "ahat_aggregate_node");
    ahat_aggregate_node node;

    ros::spin();

    return (0);
}
