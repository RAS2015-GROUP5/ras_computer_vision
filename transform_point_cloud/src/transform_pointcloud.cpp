#include "ros/ros.h"

// Ros messages
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/common/transforms.h>

ros::Subscriber pointcloud_sub;
ros::Publisher downsample_pub;
pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
pcl::PassThrough<pcl::PointXYZ> pass;
pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
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
    /*pass.setFilterFieldName ("x");
    pass.setFilterLimits (-1, 1);*/
    pass.setInputCloud (cloud_p);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 3.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_p);

    // Apply Statistical Noise Filter
    sor.setInputCloud (cloud_p);
    sor.filter (*cloud_p);

    float theta = -1.5708;
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));
    pcl::transformPointCloud (*cloud_p, *cloud_p, transform);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_p, output);


    // Publish the data
    downsample_pub.publish(output);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "transform");
    ros::NodeHandle n;

    vox.setLeafSize (0.05, 0.05, 0.05);

    sor.setMeanK (10);
    sor.setStddevMulThresh (0.1);

    downsample_pub = n.advertise<sensor_msgs::PointCloud2>("/downsample",10);
    pointcloud_sub = n.subscribe("/camera/depth/points",1, cloudCallback);
    ros::Rate loop_rate(10);
    while(ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
