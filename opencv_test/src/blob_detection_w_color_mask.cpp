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
Mat img;
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

int RLowH = 160;
int RHighH = 179;

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

    img = cv_ptr->image;


    //for ( int i = 1; i < g_sigma; i = i + 2 ){
    //}
    cv::cvtColor(img, imgHSV, COLOR_BGR2HSV);

    //RED OBJECTS

    cv::inRange(imgHSV,  cv::Scalar(RLowH, 135, 135), cv::Scalar(RHighH, 255, 255), imgHSV); //Threshold the image
    //morphological opening (remove small objects from the foreground)
    cv::erode(imgHSV, imgHSV, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));
    cv::dilate( imgHSV, imgHSV, getStructuringElement(MORPH_ELLIPSE, Size(8,8)));

    //morphological closing (fill small holes in the foreground)
    cv::dilate(imgHSV, imgHSV, getStructuringElement(MORPH_ELLIPSE, Size(8, 1)) );
    cv::erode(imgHSV, imgHSV, getStructuringElement(MORPH_ELLIPSE, Size(8, 1)) );
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

        params.filterByColor = true;
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
        RNG rng(12345);
        // Detect blobs.
        if(imgSet) {
            vector<vector<Point> > contours;
            vector<Vec4i> hierarchy;
            GaussianBlur( imgHSV, imgHSV, Size(2*g_size+1, 2*g_size+1), g_sigma, 0 );

            findContours( imgHSV, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
            vector<Rect> boundRect( contours.size() );
            vector<vector<Point> > contours_poly( contours.size() );
            for( int i = 0; i < contours.size(); i++ ) {
                approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
                boundRect[i] = boundingRect( Mat(contours_poly[i]) );
            }
            for( int i = 0; i< contours.size(); i++ )
            {
                Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
                rectangle( img, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
                imshow("Obj " + i, img(boundRect[i]));
            }
            imshow("keypoints", img);
            waitKey(3);
        }
        loop_rate.sleep();
    }
    return 0;
}

