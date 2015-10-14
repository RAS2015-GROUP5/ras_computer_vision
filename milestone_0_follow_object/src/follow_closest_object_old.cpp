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
#include <cmath>

geometry_msgs::Twist twist;
sensor_msgs::PointCloud2 output;

float min_dist = 0.30f;
float max_dist = 0.65f;

ros::Publisher pub;
bool prevWasZero = true;
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*cloud_msg, *cloud);
    //pcl_conversions::toPCL(*cloud_msg, cloud);


    float closest = 999999.0f;
    int middle = cloud->width/2;
    int ind = middle;
    float y = 0.0f;


    for(int i = 0; i < cloud->width; i++) {
        for(int j = 0; j < cloud->height;j++) {
            pcl::PointXYZ point = cloud->at(i,j);
            if(point.z < closest && point.z < max_dist) {
                closest = point.z;
                ind = i;
                y = point.x;
            }
        }
    }
    float angle = 0.0;
    if (closest > min_dist && closest < max_dist) {
        if(!prevWasZero) {
            twist.linear.x = 0.2;//0.9*(closest - min_dist);
            if(abs(abs(ind)-middle) > 0.3f*cloud->width) {
                angle = ((ind > 0) - (ind < 0))*0.1*3.14159;
            }
            else {
                angle = 0;
            }
            //angle = 0.1f*3.14159f/2.0f*((float)(ind - middle)/(float)cloud->width);
        }
        else {
            prevWasZero = false;
        }
    }

    else{
        prevWasZero = true;
        twist.linear.x = 0.0f;
    }
    twist.angular.z = angle;


    ROS_INFO("Closest distance : %f", closest);

    //ROS_INFO("Closest point on width: %i", ind);
    // Convert PointCloud2 to PCL?
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "follow_closest_object");
    ros::NodeHandle n;

    ros::Publisher robotTwist_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 10);
    ros::Subscriber point_cloud_sub = n.subscribe("/camera/depth/points", 1, pointCloudCallback);
    pub = n.advertise<sensor_msgs::PointCloud2> ("/follow_closest_object/output", 1);

    ros::Rate loop_rate(10);

    while(ros::ok()) {
        robotTwist_pub.publish(twist);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
