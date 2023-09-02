// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
// tf includes
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/icp.h>
// pcl super4PCS wrapper
#include <pcl/registration/super4pcs.h>



// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;



class super4pcs_registration_node
{
public:
    super4pcs_registration_node():
        nh{},
        cloud_sub(nh.subscribe("cloud_in", 1, &super4pcs_registration_node::cloud_cb, this)),
        cloud_pub(nh.advertise<sensor_msgs::PointCloud2>("cloud_out", 1)),
        tf_buffer(),
        tf_listener(tf_buffer),
        tf_broadcaster()
    {
        load_robot();
        // allow the tf buffer to fill
        ros::Duration(1.0).sleep();
        build_robot_cloud();
        publish_cloud(robot_cloud, "map");
    };
    
    ~super4pcs_registration_node()
    {
        ;
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub;
    
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
    PointCloudT::Ptr transformed_link_cloud;


    void cloud_cb(sensor_msgs::PointCloud2 cloud_in)
    {
        build_robot_cloud();

        // Point clouds
        PointCloudT::Ptr aligned_cloud(new PointCloudT);
        PointCloudT::Ptr scene(new PointCloudT);

        
        // load cloud
        ROS_INFO("Loading Cloud");
        pcl::fromROSMsg(cloud_in, *scene);

        // Downsample
        ROS_INFO("Downsampling Clouds");
        downsample(scene, 0.02);
        downsample(robot_cloud, 0.02);

        // Estimate normals
        ROS_INFO("Estimating Normals");
        est_normals(scene, 0.075);
        est_normals(robot_cloud, 0.075);

        // Perform super4pcs alignment
        ROS_INFO("Aligning");
        pcl::Super4PCS<PointNT, PointNT> align; 
        align.setInputSource(scene);
        align.setInputTarget(robot_cloud);
        align.setOverlap(0.8);
        align.setDelta(0.1);
        align.setSampleSize(200);
        align.align(*aligned_cloud);

        // // perform ICP refinement
        // pcl::IterativeClosestPoint<PointNT, PointNT> icp;
        // Eigen::Matrix4f estimate = Eigen::Matrix4f::Identity();
        // icp.setInputSource(aligned_cloud);
        // icp.setInputTarget(robot_cloud);
        // // icp.setMaxCorrespondenceDistance(0.05);
        // // icp.setTransformationEpsilon(1e-8);
        // // icp.setEuclideanFitnessEpsilon(1e-8);
        // icp.setMaximumIterations(10000);
        // icp.align(*aligned_cloud, estimate);

        // get the transform matrix Eigen
        Eigen::Matrix4f tf_mat = align.getFinalTransformation();
        // cast to type that tf2_eigen accepts
        Eigen::Affine3d transformation;  
        transformation.matrix() = tf_mat.cast<double>();

        // create transform TF
        geometry_msgs::TransformStamped object_alignment_tf;
        object_alignment_tf = tf2::eigenToTransform(transformation);
        object_alignment_tf.header.stamp = ros::Time::now();
        object_alignment_tf.header.frame_id = "map";
        object_alignment_tf.child_frame_id = "aligned";
        // broadcast
        tf_broadcaster.sendTransform(object_alignment_tf);

        // Publish ros msg cloud
        publish_cloud(robot_cloud, "map");
        publish_cloud(aligned_cloud, "map");
    }

    void load_robot()
    {
        // load clouds
        base_link_cloud = PointCloudT::Ptr(new PointCloudT);
        link_1_cloud = PointCloudT::Ptr(new PointCloudT);
        link_2_cloud = PointCloudT::Ptr(new PointCloudT);
        link_3_cloud = PointCloudT::Ptr(new PointCloudT);
        link_4_cloud = PointCloudT::Ptr(new PointCloudT);
        link_5_cloud = PointCloudT::Ptr(new PointCloudT);
        link_6_cloud = PointCloudT::Ptr(new PointCloudT);
        pcl::io::loadPCDFile<PointNT>("/home/lachl/robot-referencing/src/registration/pcd_files/base_link.pcd", *base_link_cloud);
        pcl::io::loadPCDFile<PointNT>("/home/lachl/robot-referencing/src/registration/pcd_files/link_1.pcd", *link_1_cloud);
        pcl::io::loadPCDFile<PointNT>("/home/lachl/robot-referencing/src/registration/pcd_files/link_2.pcd", *link_2_cloud);
        pcl::io::loadPCDFile<PointNT>("/home/lachl/robot-referencing/src/registration/pcd_files/link_3.pcd", *link_3_cloud);
        pcl::io::loadPCDFile<PointNT>("/home/lachl/robot-referencing/src/registration/pcd_files/link_4.pcd", *link_4_cloud);
        pcl::io::loadPCDFile<PointNT>("/home/lachl/robot-referencing/src/registration/pcd_files/link_5.pcd", *link_5_cloud);
        pcl::io::loadPCDFile<PointNT>("/home/lachl/robot-referencing/src/registration/pcd_files/link_6.pcd", *link_6_cloud);
    }

    void build_robot_cloud()
    {
        ROS_INFO("Building Robot Cloud");

        robot_cloud = PointCloudT::Ptr(new PointCloudT);
        transformed_link_cloud = PointCloudT::Ptr(new PointCloudT);

        for (int link_index = 1; link_index <= 6; ++link_index) 
        {
            std::string link_name = "link_" + std::to_string(link_index);

            geometry_msgs::TransformStamped tf_link;
            tf_link = tf_buffer.lookupTransform("base_link", link_name, ros::Time(0));

            Eigen::Vector3f t_link(tf_link.transform.translation.x, tf_link.transform.translation.y, tf_link.transform.translation.z);
            Eigen::Quaternionf q_link(tf_link.transform.rotation.w, tf_link.transform.rotation.x, tf_link.transform.rotation.y, tf_link.transform.rotation.z);

            switch (link_index)
            {
                case 1:
                    pcl::transformPointCloud(*link_1_cloud, *transformed_link_cloud, t_link, q_link);
                    break;
                case 2:
                    pcl::transformPointCloud(*link_2_cloud, *transformed_link_cloud, t_link, q_link);
                    break;
                case 3:
                    pcl::transformPointCloud(*link_3_cloud, *transformed_link_cloud, t_link, q_link);
                    break;
                case 4:
                    pcl::transformPointCloud(*link_4_cloud, *transformed_link_cloud, t_link, q_link);
                    break;
                case 5:
                    pcl::transformPointCloud(*link_5_cloud, *transformed_link_cloud, t_link, q_link);
                    break;
                case 6:
                    pcl::transformPointCloud(*link_6_cloud, *transformed_link_cloud, t_link, q_link);
                    break;
            }

            // *robot_cloud += *transformed_link_cloud;
        }
        *robot_cloud += *base_link_cloud;
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

    void icp_registration(PointCloudT::Ptr &scene, PointCloudT::Ptr &robot_cloud, PointCloudT::Ptr &aligned_cloud)
    {
        // replace cloud_cb with a function
        ;
    }

    void publish_cloud(PointCloudT::Ptr &cloud, std::string frame_id)
    {
        sensor_msgs::PointCloud2 ros_cloud;
        pcl::toROSMsg(*cloud, ros_cloud);
        ros_cloud.header.frame_id = frame_id;
        // Publish clouds
        cloud_pub.publish(ros_cloud);
    }

};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "super4pcs_registration_node");
    // ros::NodeHandle nh;

    // // setup pubs
    // object_aligned_pub = nh.advertise<sensor_msgs::PointCloud2>("object_aligned", 1);

    // // setup sub
    // ros::Subscriber sub = nh.subscribe("detected_cloud", 1, register_object_cb);

    super4pcs_registration_node node;

    ros::spin();

    return (0);
}
