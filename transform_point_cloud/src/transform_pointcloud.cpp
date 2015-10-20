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

#include <math.h>

ros::Subscriber pointcloud_sub;
ros::Publisher downsample_pub;
pcl::VoxelGrid<pcl::PointXYZRGB> vox;
pcl::PassThrough<pcl::PointXYZRGB> pass;
pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_floor (new pcl::PointCloud<pcl::PointXYZRGB>);

float primeSenseAngle = -0.523599;
bool floorIsFound = false;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // Instantiate various clouds
    pcl::fromROSMsg (*cloud_msg, *cloud);

    // Apply Voxel Filter on PCLPointCloud2
    vox.setInputCloud (cloud);
    vox.filter (*cloud_f);

    // Apply Passthrough Filter
    pass.setInputCloud (cloud_f);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 3.0);
    pass.filter(*cloud_f);
    pass.setInputCloud (cloud_f);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-0.085, 0.085);
    pass.filter (*cloud_f);

    //Extract floor plane
   /* pass.setInputCloud(cloud_f);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.3, 0.25);
    pass.filter(cloud_floor);*/

    // Apply Statistical Noise Filter
    sor.setInputCloud (cloud_f);
    sor.filter (*cloud_f);

    if(!floorIsFound) {
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.01);

        seg.setInputCloud (cloud_f);
        seg.segment (*inliers, *coefficients);
        std::cout << coefficients->values[0] << " " << coefficients->values[1] << " " << coefficients->values[2] << "\n";
        Eigen::Vector3f floor_normal (coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        primeSenseAngle = -acos(Eigen::Vector3f(0.0,0.0,1.0).dot(floor_normal));
        //floorIsFound = true;
        ROS_INFO("primeSenseAngle: %f", primeSenseAngle);
    }

    // Transform pointcloud to world coordinates
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate (Eigen::AngleAxisf (primeSenseAngle, Eigen::Vector3f::UnitX()));
    pcl::transformPointCloud (*cloud_f, *cloud_f, transform);
    pcl::transformPointCloud (*cloud, *cloud, transform);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_f, output);

    // Publish the data
    downsample_pub.publish(output);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "transform");
    ros::NodeHandle n;

    vox.setLeafSize (0.01, 0.01, 0.01);

    sor.setMeanK (10);
    sor.setStddevMulThresh (0.1);

    downsample_pub = n.advertise<sensor_msgs::PointCloud2>("/downsample",10);
    pointcloud_sub = n.subscribe("/camera/depth_registered/points",1, cloudCallback);
    ros::Rate loop_rate(10);
    while(ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
