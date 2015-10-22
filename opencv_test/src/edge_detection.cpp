#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;

Mat imgHSV;

int lowThreshold;
int ratio = 3;
int kernel_size = 3;
int g_sigma = 1;
int g_size = 1;
bool imgSet = false;
ros::Subscriber image_sub;
int alpha = 1;
int beta = 255;
int scale = 1;
int delta = 0;
int ddepth = CV_16S;

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

    cv::cvtColor(cv_ptr->image, imgHSV, COLOR_BGR2GRAY);
    imgSet = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "edgedetection");
    ros::NodeHandle n;
    image_sub = n.subscribe("/camera/rgb/image_raw", 1, imageCallback);
    ros::Rate loop_rate(10);

    namedWindow("Settings", 1);

    createTrackbar("Low Threshold", "Settings", &lowThreshold, 1000, NULL);
    createTrackbar("Ratio", "Settings", &ratio, 100, NULL);
    //createTrackbar("Kernel Size", "Settings", &kernel_size, 50, NULL);
    createTrackbar("Gaussian Blur sigma", "Settings", &g_sigma, 100, NULL);
    createTrackbar("Gaussian Blur size", "Settings", &g_size, 100, NULL);
    createTrackbar("Contrast", "Settings", &alpha, 200, NULL);
    createTrackbar("Brightness", "Settings", &beta, 512, NULL);
    while(ros::ok()) {
        ros::spinOnce();
        if(imgSet) {
            Mat grad;
            Mat grad_x;
            Mat grad_y;
            Mat abs_grad_x;
            Mat abs_grad_y;
            Mat new_image = Mat::zeros( imgHSV.size(), imgHSV.type() );
            GaussianBlur( imgHSV, imgHSV, Size(2*g_size+1, 2*g_size+1), g_sigma, 0 );
            double a = ((double) (alpha + 100))/100.0;
            double b = (double) beta-255;
            for( int y = 0; y < imgHSV.rows; y++ ){
                for( int x = 0; x < imgHSV.cols; x++ ){
                    imgHSV.at<uchar>(y,x) = saturate_cast<uchar>( a*( imgHSV.at<uchar>(y,x) ) + b);
                }
            }

            /// Gradient X
            //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
            Sobel( imgHSV, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
            convertScaleAbs( grad_x, abs_grad_x );

            /// Gradient Y
            //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
            Sobel( imgHSV, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
            convertScaleAbs( grad_y, abs_grad_y );

            /// Total Gradient (approximate)
            addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

            Canny( imgHSV, new_image, lowThreshold, lowThreshold*ratio, kernel_size );

            imshow("Original", imgHSV);
            imshow( "Canny", new_image );
            imshow( "Sobel", grad );
            waitKey(3);
        }
        loop_rate.sleep();
    }
    return 0;
}
