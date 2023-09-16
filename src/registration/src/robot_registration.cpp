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
#include <pcl/filters/statistical_outlier_removal.h>

// Segmentation
#include <pcl/segmentation/extract_clusters.h>

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
        aligned_cloud_pub(nh.advertise<sensor_msgs::PointCloud2>("cloud_out", 1)),
        robot_cloud_pub(nh.advertise<sensor_msgs::PointCloud2>("robot_cloud", 1)),
        tf_buffer(),
        tf_listener(tf_buffer),
        tf_broadcaster()
    {
        load_robot();
        // allow the tf buffer to fill
        ros::Duration(1.0).sleep();
        build_robot_cloud();
        robot_cloud_pub.publish(pcl_to_ros_cloud(robot_cloud, "base_link"));

        // get params
        get_parameters();

        // this will be updated to listen for an estimate provided by unity. This estimate works for the bag file
        // Eigen::Vector3f t(0, 1.044, 0.673);
        // Eigen::Quaternionf q(0.7042866369699766, 0.7050394449897112, 0.08298805706486052, 0.0035871740503676595);

        // updated guess
        Eigen::Vector3f t(0.108, 1.099, 0.680);
        Eigen::Quaternionf q(0.705, 0.709, -0.018, -0.021);
        transformation_estimate.setIdentity();
        transformation_estimate.block<3,3>(0,0) = q.toRotationMatrix();
        transformation_estimate.block<3,1>(0,3) = t;
        
        alignment_transform.setIdentity();
    };
    
    ~robot_registration_node()
    {
        ;
    }

private:
    // ROS
    ros::NodeHandle nh;
    ros::Subscriber cloud_sub;
    ros::Publisher aligned_cloud_pub;
    ros::Publisher robot_cloud_pub;
    
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
    Eigen::Matrix4f alignment_transform;

    // params
    // process
    bool use_super4pcs;
    bool use_fpfh;
    bool use_icp;
    // preprocess
    float downsample_leaf_size;
    float normal_radius_search;
    float feature_radius_search;
    int min_cluster_size;
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
        // PointCloudT::Ptr robot_aligned(new PointCloudT);
        PointCloudT::Ptr scene(new PointCloudT);
        FeatureCloudT::Ptr robot_features(new FeatureCloudT);
        FeatureCloudT::Ptr scene_features(new FeatureCloudT);

        // load cloud
        ROS_INFO("Loading Cloud");
        pcl::fromROSMsg(cloud_in, *scene);

        // Downsample
        ROS_INFO("Downsampling Cloud");
        downsample(scene, downsample_leaf_size);

        // Extract largest cluster
        ROS_INFO("Extracting Largest Cluster");
        int clusters = cluster_extraction(scene, downsample_leaf_size*1.2);
        if (clusters == 0) return;

        // Remove outliers
        ROS_INFO("Removing Outliers");
        remove_outliers(scene, 25, 1.0);

        // Perform alignment
        if (use_super4pcs) 
        {
            // Estimate normals
            ROS_INFO("Estimating Normals");
            est_normals(scene, normal_radius_search);
            est_normals(robot_cloud, normal_radius_search);

            super4pcs_register(scene);
        }

        if (use_fpfh)
        {
            // Estimate normals
            ROS_INFO("Estimating Normals");
            est_normals(scene, normal_radius_search);
            est_normals(robot_cloud, normal_radius_search);

            // Estimate features
            ROS_INFO("Estimating Features");
            est_features(robot_cloud, robot_features, feature_radius_search);
            est_features(scene, scene_features, feature_radius_search);

            fpfh_register(scene, scene_features, robot_features);
        } 

        if (use_icp) icp_refine(scene);

        // Transform cloud FOR TESTING
        ROS_INFO("Transforming Cloud");
        // pcl::transformPointCloud(*scene, *scene, transformation_estimate);

        // Broadcast tf
        broadcast_tf("world", "aligned");

        // Publish scene cloud
        aligned_cloud_pub.publish(pcl_to_ros_cloud(scene, "world"));
        // Publish robot cloud
        robot_cloud_pub.publish(pcl_to_ros_cloud(robot_cloud, "base_link"));

        // set the estimate to the most recent alignment
        // transformation_estimate = alignment_transform;

        ROS_INFO("Finished\n");
    }

    void super4pcs_register(PointCloudT::Ptr &source_cloud)
    {
        // Perform super4pcs alignment
        ROS_INFO("Aligning Super4pcs");

        pcl::Super4PCS<PointNT, PointNT> super4pcs; 
        super4pcs.setInputSource(source_cloud);
        super4pcs.setInputTarget(robot_cloud);
        super4pcs.setOverlap(overlap);
        super4pcs.setDelta(delta);
        super4pcs.setSampleSize(super4pcs_samples);
        super4pcs.align(*source_cloud, transformation_estimate);
        alignment_transform = super4pcs.getFinalTransformation();
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
        ROS_INFO("Aligning FPFH");
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
        fpfh.align(*source_cloud, transformation_estimate);
        alignment_transform = fpfh.getFinalTransformation();
    }

    void icp_refine(PointCloudT::Ptr &source_cloud)
    {
        // perform ICP refinement
        ROS_INFO("Aligning ICP");

        pcl::IterativeClosestPoint<PointNT, PointNT> icp;
        icp.setInputSource(source_cloud);
        icp.setInputTarget(robot_cloud);
        // icp.setMaxCorrespondenceDistance(max_correspondence_distance_icp);
        // icp.setTransformationEpsilon(transformation_epsilon);
        // icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
        icp.setMaximumIterations(max_icp_iterations);
        icp.align(*source_cloud, transformation_estimate);
        alignment_transform = icp.getFinalTransformation();
    }

    void load_robot()
    {
        // get dir
        std::string dir;
        ros::param::get("/registration_node/pcd_dir", dir);
        // load clouds
        base_link_cloud = PointCloudT::Ptr(new PointCloudT);
        link_1_cloud = PointCloudT::Ptr(new PointCloudT);
        link_2_cloud = PointCloudT::Ptr(new PointCloudT);
        link_3_cloud = PointCloudT::Ptr(new PointCloudT);
        link_4_cloud = PointCloudT::Ptr(new PointCloudT);
        link_5_cloud = PointCloudT::Ptr(new PointCloudT);
        link_6_cloud = PointCloudT::Ptr(new PointCloudT);
        pcl::io::loadPCDFile<PointNT>(dir+"base_link.pcd", *base_link_cloud);
        pcl::io::loadPCDFile<PointNT>(dir+"link_1.pcd", *link_1_cloud);
        pcl::io::loadPCDFile<PointNT>(dir+"link_2.pcd", *link_2_cloud);
        pcl::io::loadPCDFile<PointNT>(dir+"link_3.pcd", *link_3_cloud);
        pcl::io::loadPCDFile<PointNT>(dir+"link_4.pcd", *link_4_cloud);
        pcl::io::loadPCDFile<PointNT>(dir+"link_5.pcd", *link_5_cloud);
        pcl::io::loadPCDFile<PointNT>(dir+"link_6.pcd", *link_6_cloud);
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

            *robot_cloud += *transformed_link_cloud;
        }
        *robot_cloud += *base_link_cloud;

        downsample(robot_cloud, downsample_leaf_size);
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

    int cluster_extraction(PointCloudT::Ptr &cloud, float sr)
    {
        // Search cloud for clusters
        pcl::search::KdTree<PointNT>::Ptr tree (new pcl::search::KdTree<PointNT>);
        tree->setInputCloud(cloud);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointNT> ec;
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
        PointCloudT::Ptr cloud_cluster(new PointCloudT);
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

    void remove_outliers(PointCloudT::Ptr &cloud, int meanK, float std)
    {
        pcl::StatisticalOutlierRemoval<PointNT> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(meanK);
        sor.setStddevMulThresh(std);
        sor.filter(*cloud);
    }

    sensor_msgs::PointCloud2 pcl_to_ros_cloud(PointCloudT::Ptr &cloud, std::string frame_id)
    {
        sensor_msgs::PointCloud2 ros_cloud;
        pcl::toROSMsg(*cloud, ros_cloud);
        ros_cloud.header.frame_id = frame_id;
        return ros_cloud;
    }

    void broadcast_tf(std::string parent_frame, std::string child_frame)
    {   
        // cast to type that tf2_eigen accepts
        Eigen::Affine3d transformation;  
        transformation.matrix() = alignment_transform.cast<double>();
        // create transform TF
        geometry_msgs::TransformStamped object_alignment_tf;
        object_alignment_tf = tf2::eigenToTransform(transformation);
        object_alignment_tf.header.stamp = ros::Time::now();
        object_alignment_tf.header.frame_id = parent_frame;
        object_alignment_tf.child_frame_id = child_frame;
        // broadcast
        tf_broadcaster.sendTransform(object_alignment_tf);
    }

    void get_parameters()
    {
        ros::param::get("/registration_node/preprocess/downsample_leaf_size", downsample_leaf_size);
        ros::param::get("/registration_node/preprocess/normal_SearchRadius", normal_radius_search);
        ros::param::get("/registration_node/preprocess/features_SearchRadius", feature_radius_search);
        ros::param::get("/registration_node/preprocess/min_cluster_size", min_cluster_size);

        ros::param::get("/registration_node/use_fpfh", use_fpfh);
        ros::param::get("/registration_node/fpfh/MaximumIterations", max_fpfh_iterations);
        ros::param::get("/registration_node/fpfh/NumberOfSamples", fpfh_samples);
        ros::param::get("/registration_node/fpfh/CorrespondenceRandomness", correspondence_randomness);
        ros::param::get("/registration_node/fpfh/SimilarityThreshold", similarity_threshold);
        ros::param::get("/registration_node/fpfh/MaxCorrespondenceDistance", max_correspondence_distance_fpfh);
        ros::param::get("/registration_node/fpfh/InlierFraction", inlier_fraction);

        ros::param::get("/registration_node/use_super4pcs", use_super4pcs);
        ros::param::get("/registration_node/super4pcs/overlap", overlap);
        ros::param::get("/registration_node/super4pcs/delta", delta);
        ros::param::get("/registration_node/super4pcs/samples", super4pcs_samples);

        ros::param::get("/registration_node/use_icp", use_icp);
        ros::param::get("/registration_node/icp/MaximumIterations", max_icp_iterations);
        ros::param::get("/registration_node/icp/MaxCorrespondenceDistance", max_correspondence_distance_icp);
        ros::param::get("/registration_node/icp/TransformationEpsilon", transformation_epsilon);
        ros::param::get("/registration_node/icp/EuclideanFitnessEpsilon", euclidean_fitness_epsilon);
    }
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_registration_node");
    robot_registration_node node;

    ros::spin();

    return (0);
}
