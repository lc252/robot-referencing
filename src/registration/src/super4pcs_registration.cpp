// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// tf includes
#include <tf/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
// PCL point cloud types
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// PCL filters
#include <pcl/filters/filter.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
// PCL Super4PCS wrapper
#include <pcl/registration/super4pcs.h>
// PCL import geometry
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
// PCL ROS
#include <pcl_conversions/pcl_conversions.h>



// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;



class super4pcs_registration_node
{
public:
    super4pcs_registration_node():
        nh{},
        cloud_sub(nh.subscribe("cloud", 1, &super4pcs_registration_node::cloud_cb, this)),
        tf_buffer(),
        tf_listener(tf_buffer),
        tf_broadcaster()
    {
        load_robot();
    };
    
    ~super4pcs_registration_node();

private:
    ros::NodeHandle nh;
    ros::Subscriber cloud_sub;
    
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    PointCloudT::Ptr scene_cloud;
    PointCloudT::Ptr robot_cloud;
    PointCloudT::Ptr base_link_cloud;
    PointCloudT::Ptr link_1_cloud;
    PointCloudT::Ptr link_2_cloud;
    PointCloudT::Ptr link_3_cloud;
    PointCloudT::Ptr link_4_cloud;
    PointCloudT::Ptr link_5_cloud;
    PointCloudT::Ptr link_6_cloud;


    void cloud_cb(sensor_msgs::PointCloud2 cloud)
    {
        ;
    }

    void load_robot()
    {
        ;
    }

    void build_robot_cloud()
    {
        ;
    }

    void downsample(PointCloudT::Ptr &cloud, float leaf_size)
    {
        pcl::VoxelGrid<PointNT> grid;
        grid.setLeafSize(leaf_size, leaf_size, leaf_size);
        grid.setInputCloud(cloud);
        grid.filter(*cloud);
    }

    void est_normals(PointCloudT::Ptr &cloud, float sr)
    {
        pcl::NormalEstimationOMP<PointNT, PointNT> nest;
        nest.setRadiusSearch(sr);
        nest.setInputCloud(cloud);
        nest.compute(*cloud);
    }

    void publish_cloud(PointCloudT::Ptr &cloud)
    {
        sensor_msgs::PointCloud2 ros_cloud;
        pcl::toROSMsg(*cloud, ros_cloud);
        ros_cloud.header.frame_id = "camera_color_optical_frame";
        // Publish clouds
        object_aligned_pub.publish(ros_cloud);
    }

};




// Publishers
ros::Publisher object_aligned_pub;

void downsample(PointCloudT::Ptr &cloud)
{
    float leaf_size;
    ros::param::get("leaf_size", leaf_size);
    pcl::VoxelGrid<PointNT> grid;
    grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    grid.setInputCloud(cloud);
    grid.filter(*cloud);
}

void est_normals(PointCloudT::Ptr &cloud)
{
    float sr;
    ros::param::get("normal_SearchRadius", sr);
    pcl::NormalEstimationOMP<PointNT, PointNT> nest;
    nest.setRadiusSearch(0.01);
    nest.setInputCloud(cloud);
    nest.compute(*cloud);
}

void publish_cloud(PointCloudT::Ptr &cloud)
{
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloud, ros_cloud);
    ros_cloud.header.frame_id = "camera_color_optical_frame";
    // Publish clouds
    object_aligned_pub.publish(ros_cloud);
}

void register_object_cb(sensor_msgs::PointCloud2 cloud)
{
    // Point clouds
    PointCloudT::Ptr object(new PointCloudT);
    PointCloudT::Ptr object_aligned(new PointCloudT);
    PointCloudT::Ptr scene(new PointCloudT);

    
    // load clouds
    ROS_INFO("Loading Clouds");
    pcl::fromROSMsg(cloud, *scene);
    pcl::io::loadPCDFile<PointNT>("/home/lachl/inference-2d-3d/src/object_detection/model_geometry/model_car_scaled_normal.pcd", *object);

    // Downsample
    ROS_INFO("Downsampling Clouds");
    downsample(scene);
    downsample(object);

    // Estimate normals
    ROS_INFO("Estimating Normals");
    est_normals(scene);
    est_normals(object);

    // Perform alignment
    ROS_INFO("Aligning");
    float overlap, delta, samples;
    ros::param::get("overlap", overlap);
    ros::param::get("delta", delta);
    ros::param::get("samples", samples);
    pcl::Super4PCS<PointNT, PointNT> align; 
    align.setInputSource(object);
    align.setInputTarget(scene);
    align.setOverlap(overlap);
    align.setDelta(delta);
    align.setSampleSize(samples);
    align.align(*object_aligned);

    // get the transform matrix Eigen
    Eigen::Matrix4f tf_mat = align.getFinalTransformation();
    // cast to type that tf2_eigen accepts
    Eigen::Affine3d transformation;  
    transformation.matrix() = tf_mat.cast<double>();

    // create transform TF
    geometry_msgs::TransformStamped object_alignment_tf;
    object_alignment_tf = tf2::eigenToTransform(transformation);
    object_alignment_tf.header.stamp = ros::Time::now();
    object_alignment_tf.header.frame_id = "camera_color_optical_frame";
    object_alignment_tf.child_frame_id = "object";
    // broadcast
    static tf::TransformBroadcaster br;
    br.sendTransform(object_alignment_tf);

    // Publish ros msg cloud
    publish_cloud(object_aligned);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "super4pcs_registration_node");
    ros::NodeHandle nh;

    // setup pubs
    object_aligned_pub = nh.advertise<sensor_msgs::PointCloud2>("object_aligned", 1);

    // setup sub
    ros::Subscriber sub = nh.subscribe("detected_cloud", 1, register_object_cb);

    ros::spin();

    return (0);
}
