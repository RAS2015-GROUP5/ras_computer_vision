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

int g_sigma = 6;
int g_size = 6;
int min_size = 0;
int max_size = 10000;
int min_inert = 100;
int max_inert = 1000;
int min_thres = 100;
int max_thres = 1000;
int min_circ = 100;
int max_circ = 1000;
int col_fil = 0;


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


    //for ( int i = 1; i < g_sigma; i = i + 2 ){
    //}

    imgSet = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "blobdetection");
    ros::NodeHandle n;
    image_sub = n.subscribe("/camera/rgb/image_raw", 1, imageCallback);
    ros::Rate loop_rate(10);

    namedWindow("Settings", 1);

    createTrackbar("Gaussian Blur sigma", "Settings", &g_sigma, 100, NULL);
    createTrackbar("Gaussian Blur size", "Settings", &g_size, 100, NULL);
    //createTrackbar("Color filter", "Settings", &col_fil, 255, NULL); //Does not work?
    createTrackbar("Min size", "Settings", &min_size, 10000, NULL);
    createTrackbar("Max size", "Settings", &max_size, 10000, NULL);
    createTrackbar("Min Inertia", "Settings", &min_inert, 1000, NULL);
    createTrackbar("Max Inertia", "Settings", &max_inert, 1000, NULL);
    createTrackbar("Min Threshold", "Settings", &min_thres, 1000, NULL);
    createTrackbar("Max Threshold", "Settings", &max_thres, 1000, NULL);
    createTrackbar("Min Circularity", "Settings", &min_circ, 1000, NULL);
    createTrackbar("Max Circularity", "Settings", &max_circ, 1000, NULL);


    while(ros::ok()) {
        ros::spinOnce();

        //params.filterByColor = true;
        //params.blobColor = col_fil;
        params.filterByArea = true;
        params.minArea = min_size + 100;
        params.maxArea = max_size + 100;
        params.filterByInertia = true;
        params.minInertiaRatio  = (float) min_inert/1000.0;
        params.maxInertiaRatio  = (float) max_inert/1000.0;
        params.minThreshold = min_thres;
        params.maxThreshold = max_thres;
        params.minCircularity = (float) min_circ/1000.0;
        params.maxCircularity = (float) max_circ/1000.0;

        // Detect blobs.
        if(imgSet) {
            GaussianBlur( imgHSV, imgHSV, Size(2*g_size+1, 2*g_size+1), g_sigma, 0 );
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
