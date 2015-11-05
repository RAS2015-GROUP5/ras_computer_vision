#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/features2d/features2d.hpp>

using namespace cv;

ros::Subscriber image_sub;
Mat imgHSV;
Mat img;

FlannBasedMatcher matcher;
//BFMatcher matcher(NORM_L2);
//Ptr<DescriptorExtractor> extractor = cv::DescriptorExtractor::create("ORB");

OrbFeatureDetector      orb(700,1.2f,8,8,0,2,0,14);   //> (From 31 to 14)
OrbDescriptorExtractor extractor(700,1.2f,8,8,0,2,0,14);

//orbFeatureDetector orb(700, 1.2f, 8, 8, 0, 2, ORB::HARRIS_SCORE, 31);
Mat rect_img;
bool imgSet = false;

int g_sigma = 6;
int g_size = 6;

int RLowH = 160;
int RHighH = 179;

int scale = 1;
int delta = 0;
int ddepth = CV_16S;
int f_threshold = 50;
int maxGoodDistance = 100000;

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
    createTrackbar("FAST threshold", "Settings", &f_threshold, 255, NULL);
    createTrackbar("Max good distance", "Settings", &maxGoodDistance, 100000, NULL);

    rect_img = imread("/home/ras25/junk/rect.png", 0);
    Mat rect_mask;
    Mat rect_mask_after;
    std::vector<KeyPoint> keypoints;
    orb.detect(rect_img, keypoints);
    drawKeypoints( rect_img, keypoints, rect_mask, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    imshow("Before", rect_mask);
    std::cout << "Keypoints: " << keypoints.size() <<"\n";
    Mat rect_descriptor;
    extractor.compute(rect_img, keypoints, rect_descriptor);
    rect_descriptor.convertTo(rect_descriptor, CV_32F);
    drawKeypoints( rect_img, keypoints, rect_mask_after, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    imshow("After", rect_mask_after);
    while(ros::ok()) {
        ros::spinOnce();
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
                //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
                //rectangle( img, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
                if(boundRect[i].size().width > 50 && boundRect[i].size().height > 50) {
                    Mat rectImg;
                    cvtColor(img(boundRect[i]), rectImg, COLOR_BGR2GRAY);
                    imshow("colorImg", rectImg);
                    Mat grad;
                    Mat grad_x;
                    Mat grad_y;
                    Mat abs_grad_x;
                    Mat abs_grad_y;
                    Sobel( rectImg, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
                    convertScaleAbs( grad_x, abs_grad_x );

                    /// Gradient Y
                    //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
                    Sobel( rectImg, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
                    convertScaleAbs( grad_y, abs_grad_y );

                    /// Total Gradient (approximate)
                    addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
                    imshow("grad", grad);
                    std::vector<KeyPoint> keypoints_2;
                    orb.detect(grad, keypoints_2);
                    if(keypoints_2.size() > 100) {
                        Mat rect_descriptor_2;
                        extractor.compute(grad, keypoints_2, rect_descriptor_2);
                        rect_descriptor_2.convertTo(rect_descriptor_2, CV_32F);
                        std::vector< DMatch > matches;
                        matcher.match( rect_descriptor, rect_descriptor_2, matches );
                        double max_dist = 0; double min_dist = 100;
                        for( int i = 0; i < rect_descriptor.rows; i++ )
                        { double dist = matches[i].distance;
                          if( dist < min_dist ) min_dist = dist;
                          if( dist > max_dist ) max_dist = dist;
                        }

                        std::vector< DMatch > good_matches;

                        for( int i = 0; i < rect_descriptor.rows; i++ )
                        { if( matches[i].distance <= max(2*(double)maxGoodDistance/1000, 0.02) )
                          { good_matches.push_back( matches[i]); }
                        }
                        std::cout << "Good matches: " << good_matches.size() << "\n";
                        Mat img_matches;
                        drawMatches( rect_img, keypoints, grad, keypoints_2,
                                     good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                                     vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

                        //imwrite( "/home/ras25/junk/rect2.png", grad );
                        imshow("rect_img", img_matches);
                    }
                    else {
                        std::cout << "Not enough keypoints\n";
                    }

                    imshow("keypoints", img);
                    waitKey(3);
                }
            }
        }
        loop_rate.sleep();
    }
    return 0;
}

