#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;

ros::Subscriber image_sub;
Mat imgHSV;
SimpleBlobDetector::Params params;
cv::SimpleBlobDetector detector;

bool imgSet = false;

int a = 1;
int b = 100;
int c = 10000;
int d = 100;
int e = 1000;
int f = 100;
int g = 1000;
int h = 100;
int l = 1000;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }

    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
    //cv::cvtColor(cv_ptr->image, imgHSV, COLOR_BGR2HSV);
    imgHSV = cv_ptr->image;

    for ( int i = 1; i < a; i = i + 2 ){
        GaussianBlur( imgHSV, imgHSV, Size( i, i ), 0, 0 );
    }

    imgSet = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "blobdetection");
    ros::NodeHandle n;
    image_sub = n.subscribe("/camera/rgb/image_raw", 1, imageCallback);
    ros::Rate loop_rate(10);
    namedWindow("Settings", 1);
    createTrackbar("Gaussian Blur", "Settings", &a, 100, NULL);
    createTrackbar("Min size", "Settings", &b, 10000, NULL);
    createTrackbar("Max size", "Settings", &c, 10000, NULL);
    createTrackbar("Min Threshold", "Settings", &d, 1000, NULL);
    createTrackbar("Max Threshold", "Settings", &e, 1000, NULL);
    //createTrackbar("Min Circularity")
    while(ros::ok()) {
        ros::spinOnce();
        params.filterByArea = true;
        params.minArea = b;
        params.maxArea = c;
        params.minThreshold = d;
        params.maxThreshold = e;
        params.minCircularity = (float) f/1000.0;
        params.maxCircularity = (float) g/1000.0;
        // Detect blobs.
        if(imgSet) {
            std::vector<KeyPoint> keypoints;
            SimpleBlobDetector detector(params);
            detector.detect( imgHSV, keypoints);

            // Draw detected blobs as red circles.
            // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
            Mat im_with_keypoints;
            drawKeypoints( imgHSV, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

            // Show blobs
            imshow("keypoints", im_with_keypoints);
            waitKey(3);
        }
        loop_rate.sleep();
    }
    return 0;
}
