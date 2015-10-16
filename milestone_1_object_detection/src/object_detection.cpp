#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>

// Reusable objects
ros::Publisher downsample_pub;
ros::Subscriber depth_sub;
pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
pcl::PassThrough<pcl::PointXYZ> pass;
pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
pcl::SACSegmentation<pcl::PointXYZ> seg;
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;


void depthPointsCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {

    // Instantiate various clouds
    pcl::PCLPointCloud2* cloud_intermediate = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_intermediate);
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud_intermediate);

    // Apply Voxel Filter on PCLPointCloud2
    vox.setInputCloud (cloudPtr);
    vox.setInputCloud (cloudPtr);
    vox.filter (*cloud_intermediate);

    // Convert PCL::PointCloud2 to PCL::PointCloud<PointXYZ>
    pcl::fromPCLPointCloud2(*cloud_intermediate, cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p = cloud.makeShared();

    // Apply Passthrough Filter
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.3, 1);
    pass.setInputCloud (cloud_p);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_p);

    // Apply Passthrough Filter
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-0.2, 0.2);
    pass.setInputCloud (cloud_p);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 3.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_p);

    // Apply Statistical Noise Filter
    sor.setInputCloud (cloud_p);
    sor.filter (*cloud_p);

    // Planar segmentation: Remove large planes? Or extract floor?
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    int nr_points = (int) cloud_p->points.size ();

    Eigen::Vector3f lol (0, 1, 0);
    seg.setEpsAngle(  30.0f * (3.14f/180.0f) );
    seg.setAxis(lol);
    //while(cloud_p->points.size () > 0.2 * nr_points) {
    sor.setInputCloud (cloud_p);
    sor.filter (*cloud_p);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (cloud_p);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        //break;
    }
    else {
        /*std::cout << "Model coefficients: " << coefficients->values[0] << " "
                      << coefficients->values[1] << " "
                      << coefficients->values[2] << " "
                      << coefficients->values[3] << "\n";*/
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_p);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud_p);
    }
    //}

    Eigen::Vector3f lol_p (0.5f, 0, 0.5f);
    seg.setAxis(lol_p);
    while(cloud_p->points.size () > 0.1 * nr_points) {

        seg.setInputCloud (cloud_p);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
            break;
        }
        else {
            /*std::cout << "Model coefficients: " << coefficients->values[0] << " "
                      << coefficients->values[1] << " "
                      << coefficients->values[2] << " "
                      << coefficients->values[3] << "\n";*/
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud_p);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*cloud_p);
        }
    }

    // Apply Statistical Noise Filter
    sor.setInputCloud (cloud_p);
    sor.filter (*cloud_p);

    if(cloud_p->points.size() > 0) {
        std::vector<pcl::PointIndices> cluster_indices;
        tree->setInputCloud (cloud_p);
        ec.setInputCloud (cloud_p);
        ec.extract (cluster_indices);

        std::cout << "Clusters detected: " << cluster_indices.size() << "\n";
        // Convert to ROS data type
    }
    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_p, output);


    // Publish the data
    downsample_pub.publish(output);

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "downsample");
    ros::NodeHandle n;
    downsample_pub = n.advertise<sensor_msgs::PointCloud2>("/downsample", 10);
    depth_sub = n.subscribe("/camera/depth/points", 10, depthPointsCallback);
    ros::Rate loop_rate(10);

    // Set variables for filters

    vox.setLeafSize (0.01, 0.01, 0.01);

    sor.setMeanK (10);
    sor.setStddevMulThresh (0.1);

    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (20);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);

    //seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.02);

    while(ros::ok()) {
        ros::spin();
    }
    return 0;
}

