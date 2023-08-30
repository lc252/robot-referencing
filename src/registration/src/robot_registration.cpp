// ROS 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// TF
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

// FPFH
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/features/normal_3d_omp.h>

// Super4PCS wrapper
#include <pcl/registration/super4pcs.h>

// ICP
#include <pcl/registration/icp.h>



// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;



class robot_registration_node
{
public:
    robot_registration_node():
        nh{},
        cloud_sub(nh.subscribe("cloud_in", 1, &robot_registration_node::cloud_cb, this)),
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

        // get params
        ros::param::get("~use_super4pcs", use_super4pcs);
        ros::param::get("~use_icp", use_icp);
        ros::param::get("~process/downsample_leaf_size", downsample_leaf_size);
        ros::param::get("~process/normal_radius_search", normal_radius_search);
        ros::param::get("~process/feature_radius_search", feature_radius_search);

        ros::param::get("~fpfh/max_fpfh_iterations", max_fpfh_iterations);
        ros::param::get("~fpfh/fpfh_samples", fpfh_samples);
        ros::param::get("~fpfh/correspondence_randomness", correspondence_randomness);
        ros::param::get("~fpfh/similarity_threshold", similarity_threshold);
        ros::param::get("~fpfh/max_correspondence_distance", max_correspondence_distance_fpfh);
        ros::param::get("~fpfh/inlier_fraction", inlier_fraction);

        ros::param::get("~super4pcs/overlap", overlap);
        ros::param::get("~super4pcs/delta", delta);
        ros::param::get("~super4pcs/super4pcs_samples", super4pcs_samples);

        ros::param::get("~icp/max_icp_iterations", max_icp_iterations);
        ros::param::get("~icp/max_correspondence_distance_icp", max_correspondence_distance_icp);
        ros::param::get("~icp/transformation_epsilon", transformation_epsilon);
        ros::param::get("~icp/euclidean_fitness_epsilon", euclidean_fitness_epsilon);

    };
    
    ~robot_registration_node()
    {
        ;
    }

private:
    // ROS
    ros::NodeHandle nh;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub;
    
    // tf
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    // clouds
    PointCloudT::Ptr scene_cloud;
    PointCloudT::Ptr robot_cloud;

    // links
    PointCloudT::Ptr base_link_cloud;
    PointCloudT::Ptr link_1_cloud;
    PointCloudT::Ptr link_2_cloud;
    PointCloudT::Ptr link_3_cloud;
    PointCloudT::Ptr link_4_cloud;
    PointCloudT::Ptr link_5_cloud;
    PointCloudT::Ptr link_6_cloud;
    PointCloudT::Ptr transformed_link_cloud;

    // transform estimation
    Eigen::Matrix4f transformation_estimate;

    // params
    // process
    bool use_super4pcs;
    bool use_icp;
    float downsample_leaf_size;
    float normal_radius_search;
    float feature_radius_search;
    // fpfh
    int max_fpfh_iterations;
    int fpfh_samples;
    int correspondence_randomness;
    float similarity_threshold;
    float max_correspondence_distance_fpfh;
    float inlier_fraction;
    // super4pcs
    float overlap;
    float delta;
    int super4pcs_samples;
    // icp
    int max_icp_iterations;
    float max_correspondence_distance_icp;
    float transformation_epsilon;
    float euclidean_fitness_epsilon;


    void cloud_cb(sensor_msgs::PointCloud2 cloud_in)
    {
        build_robot_cloud();

        // Point clouds
        PointCloudT::Ptr robot_aligned(new PointCloudT);
        PointCloudT::Ptr scene(new PointCloudT);
        FeatureCloudT::Ptr robot_features(new FeatureCloudT);
        FeatureCloudT::Ptr scene_features(new FeatureCloudT);

        
        // load cloud
        ROS_INFO("Loading Cloud\n");
        pcl::fromROSMsg(cloud_in, *scene);

        // Downsample
        ROS_INFO("Downsampling Clouds\n");
        downsample(scene, downsample_leaf_size);
        downsample(robot_cloud, downsample_leaf_size);

        // Estimate normals
        ROS_INFO("Estimating Normals\n");
        est_normals(scene, normal_radius_search);
        est_normals(robot_cloud, normal_radius_search);

        // Estimate features
        est_features(robot_cloud, robot_features, feature_radius_search);
        est_features(scene, scene_features, feature_radius_search);

        // Perform alignment
        if (use_super4pcs)
        {
            super4pcs_register(scene);
        }
        else
        {
            fpfh_register(scene, scene_features, robot_features);
        }

        if (use_icp)
        {
            icp_refine(scene);
        }

        // Broadcast tf
        broadcast_tf("map", "aligned");

        // Publish ros msg cloud
        publish_cloud(scene, "map");
    }

    void super4pcs_register(PointCloudT::Ptr &source_cloud)
    {
        // Perform super4pcs alignment
        ROS_INFO("Aligning Super4pcs\n");

        pcl::Super4PCS<PointNT, PointNT> super4pcs; 
        super4pcs.setInputSource(source_cloud);
        super4pcs.setInputTarget(robot_cloud);
        super4pcs.setOverlap(overlap);
        super4pcs.setDelta(delta);
        super4pcs.setSampleSize(super4pcs_samples);
        super4pcs.align(*source_cloud);
        transformation_estimate = super4pcs.getFinalTransformation();
    }

    void fpfh_register(PointCloudT::Ptr &source_cloud, FeatureCloudT::Ptr &source_features, FeatureCloudT::Ptr &robot_features)
    {
        // Perform FPFH alignment
        /* Parameters
         * MaximumIterations: Number of RANSAC iterations, increase to tradeoff speed for accuracy (50000)
         * NumberOfSamplesNumber: Number of points to sample for generating/prerejecting a pose, increase to tradeoff speed for accuracy (3)
         * CorrespondenceRandomness: Number of nearest features to use, increase to tradeoff speed for accuracy (5)
         * setSimilarityThreshold: Polygonal edge length similarity threshold, decrease to tradeoff speed for accuracy (0.9)
         * setMaxCorrespondenceDistance: Inlier threshold (2.5 * leaf size)
         * setInlierFraction: Required inlier fraction for accepting a pose hypothesis, increase  (0.25)
         */
        ROS_INFO("Aligning FPFH\n");

        pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> fpfh;
        fpfh.setInputSource(source_cloud);
        fpfh.setSourceFeatures(source_features);
        fpfh.setInputTarget(robot_cloud);
        fpfh.setTargetFeatures(robot_features);
        fpfh.setMaximumIterations(max_fpfh_iterations);
        fpfh.setNumberOfSamples(fpfh_samples);
        fpfh.setCorrespondenceRandomness(correspondence_randomness);
        fpfh.setSimilarityThreshold(similarity_threshold);
        fpfh.setMaxCorrespondenceDistance(max_correspondence_distance_fpfh);
        fpfh.setInlierFraction(inlier_fraction);
        fpfh.align(*source_cloud);
        transformation_estimate = fpfh.getFinalTransformation();
    }

    void icp_refine(PointCloudT::Ptr &source_cloud)
    {
        // perform ICP refinement
        ROS_INFO("Aligning ICP\n");

        pcl::IterativeClosestPoint<PointNT, PointNT> icp;
        icp.setInputSource(source_cloud);
        icp.setInputTarget(robot_cloud);
        icp.setMaxCorrespondenceDistance(max_correspondence_distance_icp);
        icp.setTransformationEpsilon(transformation_epsilon);
        icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
        icp.setMaximumIterations(max_icp_iterations);
        icp.align(*source_cloud);
        transformation_estimate = icp.getFinalTransformation();
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
        ROS_INFO("Building Robot Cloud\n");

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

    void est_features(PointCloudT::Ptr &cloud, FeatureCloudT::Ptr &features, float sr)
    {
        FeatureEstimationT fest;
        fest.setRadiusSearch(sr);
        fest.setInputCloud(cloud);
        fest.setInputNormals(cloud);
        fest.compute(*features);
    }

    void publish_cloud(PointCloudT::Ptr &cloud, std::string frame_id)
    {
        sensor_msgs::PointCloud2 ros_cloud;
        pcl::toROSMsg(*cloud, ros_cloud);
        ros_cloud.header.frame_id = frame_id;
        // Publish clouds
        cloud_pub.publish(ros_cloud);
    }

    void broadcast_tf(std::string parent_frame, std::string child_frame)
    {
        // cast to type that tf2_eigen accepts
        Eigen::Affine3d transformation;  
        transformation.matrix() = transformation_estimate.cast<double>();
        // create transform TF
        geometry_msgs::TransformStamped object_alignment_tf;
        object_alignment_tf = tf2::eigenToTransform(transformation);
        object_alignment_tf.header.stamp = ros::Time::now();
        object_alignment_tf.header.frame_id = parent_frame;
        object_alignment_tf.child_frame_id = child_frame;
        // broadcast
        tf_broadcaster.sendTransform(object_alignment_tf);
    }

};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_registration_node");
    robot_registration_node node;

    ros::spin();

    return (0);
}
