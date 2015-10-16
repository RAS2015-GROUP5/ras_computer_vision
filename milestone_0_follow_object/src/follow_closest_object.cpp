/**
  This code reads an image from /camera/depth/image_raw ?? and publishes a geometry_msgs/Twist to /RobotTwist
  */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <std_msgs/Float64.h>

geometry_msgs::Twist twist;
sensor_msgs::PointCloud2 output;

float min_dist = 0.2f;
float max_dist = 0.9f;
float clr_dist = 0.35f;

ros::Publisher pub;
//ros::Publisher depth_pub;
//ros::Subscriber depth_sub;

bool prevWasZero = true;

float saturate(float val, float min, float max) {
    return std::min(std::max(val, min), max);
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*cloud_msg, *cloud);
    //pcl_conversions::toPCL(*cloud_msg, cloud);

    pcl::PassThrough<pcl::PointXYZ> pass;


    // Filter Z coordinates larger than 0.75
    if(!(cloud->empty())) {
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (min_dist, max_dist);
        pass.filter(*cloud_filtered);
    }

    // Filter Y coordinates on the ground?
    if(!(cloud_filtered->empty())) {
        pass.setInputCloud (cloud_filtered);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-1, 0.12);
        pass.filter(*cloud_filtered);
    }

    pcl::PointXYZ min_p, max_p;

    Eigen::Vector4f centroid(0.0f, 0.0f, 99999.0f, 0.0f);

    pcl::getMinMax3D(*cloud_filtered, min_p, max_p);

    //pcl::PassThrough<pcl::PointXYZ> pass2;

    // Filter Z coordinates larger than 0.75
    if(!(cloud_filtered->empty())) {
        pass.setInputCloud (cloud_filtered);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (min_p.z-0.02, min_p.z+0.03);
        pass.filter(*cloud_filtered);
    }

    
    /*/Filter outliers (Statistical) -- SLOW!
    if(!(cloud_filtered->empty())) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud_filtered);
        sor.setMeanK (100);
        sor.setStddevMulThresh (0.0000001);
        sor.filter (*cloud_filtered);
    }*/

    /*if(!(cloud_filtered->empty())) {
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(cloud_filtered);
    outrem.setRadiusSearch(0.8);
    outrem.setMinNeighborsInRadius (1000);
    // apply filter
    outrem.filter (*cloud_filtered);
    }*/

    // Check if closest point is valid why not?
    twist.angular.z = 0.0;
    twist.linear.x = 0.0;

    //pcl::copyPointCloud(*cloud_filtered, *cloud_pub); 

    //std::cout<< "No. of points in the cloud : " << cloud_pub->points.size() << std::endl;
    /*
    if(cloud_filtered->points.size() < 700) {
        cloud_filtered->points.clear();
    }
    */
    //if(!(cloud_filtered->empty())) {
    if(cloud_filtered->points.size() > 700) {
        pcl::compute3DCentroid (*cloud_filtered, centroid);
        twist.angular.z = saturate(0.4*(3.14159f/2.0f*(centroid[0]/0.24f)), -0.15, 0.15);
        //twist.angular.z = 0.2*(3.14159f/2.0f*(centroid[0]/0.24f));
        twist.linear.x = 0.7*(centroid[2]-clr_dist);
        
        //std::cout << "centroid:" << centroid[0] << " " <<  centroid[1] << " " <<   centroid[2] << " " <<   centroid[3] << " \n";
    }
    else {  
    }


    //std::cout<< "No. of points in the cloud : " << cloud_pub->points.size() << std::endl;

    /*/ Filter outliers (Nearest Neighbors) -- HALTS PROGRAM WTF
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud_filtered);
    outrem.setRadiusSearch(0.5);
    outrem.setMinNeighborsInRadius (8);
    outrem.filter (*cloud_filtered);
    */

    /*
    float closest = 999999.0f;
    int middle = cloud->width/2;
    int ind = middle;
    float y = 0.0f;
    */

    /*for(int i = 0; i < cloud->width; i++) {
        for(int j = 0; j < cloud->height;j+=2) {
            pcl::PointXYZ point = cloud->at(i,j);
            if(point.z < closest && point.z < 0.75) {
                closest = point.z;
                ind = i;
                y = point.x;
            }
        }
    }
    */
    ROS_INFO("Closest point is at : %f : %f : ", centroid[2], min_p.z);

    sensor_msgs::PointCloud2 output;
    //pcl_conversions::fromPCL(cloud_filtered, output);
    pcl::toROSMsg(*cloud_filtered, output);
    pub.publish (output);
        
    
    /*
    if (closest > min_dist && closest < 0.75f) {
        if(!prevWasZero) {
            twist.linear.x = 0.75*(closest - min_dist);
            angle = 3.14159f/2.0f*((float)(ind - middle)/(float)cloud->width);
        }
        else {
            prevWasZero = false;
        }
    }

    else{
        prevWasZero = true;
        twist.linear.x = 0.0f;
    }
    */

    //ROS_INFO("Closest point on width: %i", ind);
    // Convert PointCloud2 to PCL?

   // 
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "follow_closest_object");
    ros::NodeHandle n;

    ros::Publisher robotTwist_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 10);
    ros::Subscriber point_cloud_sub = n.subscribe("/camera/depth/points", 1, pointCloudCallback);
    pub = n.advertise<sensor_msgs::PointCloud2> ("/follow_closest_object/output", 1);
    //depth_pub = n.advertise<std_msgs::Float64> ("/follow_closest_object/closest", 1);
    //depth_sub = n.subscribe("/follow_closest_object/closest", 1);

    ros::Rate loop_rate(10);

    while(ros::ok()) {
        ros::spinOnce();
        robotTwist_pub.publish(twist);
        loop_rate.sleep();
    }
    return 0;
}
