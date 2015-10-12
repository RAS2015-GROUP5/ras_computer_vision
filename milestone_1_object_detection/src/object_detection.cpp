#include "ros/ros.h";
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


ros::Publisher downsample_pub;
ros::Subscriber depth_sub;

void depthPointsCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {

    // Instantiate various clouds
    pcl::PCLPointCloud2* cloud_intermediate = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_intermediate);
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud_intermediate);

    // Apply Voxel Filter on PCLPointCloud2
    pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
    vox.setInputCloud (cloudPtr);
    vox.setLeafSize (0.025, 0.025, 0.025);
    vox.filter (*cloud_intermediate);

    // Convert PCL::PointCloud2 to PCL::PointCloud<PointXYZ>
    pcl::fromPCLPointCloud2(*cloud_intermediate, cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p = cloud.makeShared();

    // Apply Passthrough Filter
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_p);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 3.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_p);

    // Apply Statistical Noise Filter
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_p);
    sor.setMeanK (5);
    sor.setStddevMulThresh (0.05);
    sor.filter (*cloud_p);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
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

    ROS_INFO("Inliers: %i", inliers->indices.size());
    if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      return (-1);
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
    while(ros::ok()) {
        ros::spin();
    }
    return 0;
}

