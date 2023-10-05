// ROS 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>



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
        ros::param::get("/registration_node/preprocess/min_cluster_size", min_cluster_size);

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
    int min_cluster_size;

    void cloud_cb(sensor_msgs::PointCloud2 cloud_in)
    {
        PointCloudXYZ::Ptr new_cloud(new PointCloudXYZ);

        // load cloud
        ROS_INFO("Loading Cloud");
        pcl::fromROSMsg(cloud_in, *new_cloud);;

        // add
        *cloud += *new_cloud;

        downsample(cloud, downsample_leaf_size);
        remove_outliers(cloud, 50, 1.0);
        cluster_extraction(cloud, downsample_leaf_size*1.2);

        // publish
        sensor_msgs::PointCloud2 cloud_out;
        pcl::toROSMsg(*cloud, cloud_out);
        cloud_out.header.frame_id = "world";
        cloud_pub.publish(cloud_out);
    }

    void downsample(PointCloudXYZ::Ptr &cloud, float leaf_size)
    {
        pcl::VoxelGrid<pcl::PointXYZ> grid;
        grid.setLeafSize(leaf_size, leaf_size, leaf_size);
        grid.setInputCloud(cloud);
        grid.filter(*cloud);
    }

    int cluster_extraction(PointCloudXYZ::Ptr &cloud, float sr)
    {
        // Search cloud for clusters
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(sr);
        ec.setMinClusterSize(min_cluster_size);
        ec.setMaxClusterSize(50000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        // Find largest cluster
        int idx = 0;
        int largest_cluster_size = 0;
        int cluster_size;
        for (int i = 0; i < cluster_indices.size(); ++i)
        {
            cluster_size = cluster_indices[i].indices.size();
            if (cluster_size > largest_cluster_size)
            {
                largest_cluster_size = cluster_size;
                idx = i;
            }
        }
        
        if (largest_cluster_size == 0)
        {
            ROS_ERROR("No clusters found");
            return 0;
        }

        // Extract largest cluster
        PointCloudXYZ::Ptr cloud_cluster(new PointCloudXYZ);
        for(std::vector<int>::const_iterator it = cluster_indices[idx].indices.begin(); it != cluster_indices[idx].indices.end(); ++it)
        {
            cloud_cluster->push_back((*cloud)[*it]);
        }
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // Return largest cluster
        *cloud = *cloud_cluster;
        return 1;
    }

    void remove_outliers(PointCloudXYZ::Ptr &cloud, int meanK, float std)
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(meanK);
        sor.setStddevMulThresh(std);
        sor.filter(*cloud);
    }

};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "ahat_aggregate_node");
    ahat_aggregate_node node;

    ros::spin();

    return (0);
}
