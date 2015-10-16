#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image Processed";

ros::Subscriber image_sub;

int OLowH = 0;
int OHighH = 10;

int YLowH = 10;
int YHighH = 35;

int GLowH = 35;
int GHighH = 75;

int BLowH = 75;
int BHighH = 130;

int VLowH = 130;
int VHighH = 160;

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

    cv::Mat imgHSV;
    cv::Mat imgThresholded;
    cv::cvtColor(cv_ptr->image, imgHSV, COLOR_BGR2HSV);

    //RED OBJECTS

    cv::inRange(imgHSV,  cv::Scalar(RLowH, 135, 135), cv::Scalar(RHighH, 255, 255), imgThresholded); //Threshold the image
    //morphological opening (remove small objects from the foreground)
    cv::erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));
    cv::dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(8,8)));

    //morphological closing (fill small holes in the foreground)
    cv::dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(8, 1)) );
    cv::erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(8, 1)) );
    cv::imshow("RED OBJECTS", imgThresholded); //show the thresholded image
    cv::waitKey(1);

    // ORANGE OBJECTS
    cv::inRange(imgHSV,  cv::Scalar(OLowH, 135, 135), cv::Scalar(OHighH, 255, 255), imgThresholded); //Threshold the image
    //morphological opening (remove small objects from the foreground)
    cv::erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));
    cv::dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(8,8)));

    //morphological closing (fill small holes in the foreground)
    cv::dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(8, 1)) );
    cv::erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(8, 1)) );
    cv::imshow("ORANGE OBJECTS", imgThresholded); //show the thresholded image
    cv::waitKey(1);

    // GREEN OBJECTS
    cv::inRange(imgHSV,  cv::Scalar(GLowH, 135, 135), cv::Scalar(GHighH, 255, 255), imgThresholded); //Threshold the image
    //morphological opening (remove small objects from the foreground)
    cv::erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));
    cv::dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(8,8)));

    //morphological closing (fill small holes in the foreground)
    cv::dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(8, 1)) );
    cv::erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(8, 1)) );
    cv::imshow("GREEN OBJECTS", imgThresholded); //show the thresholded image
    cv::waitKey(1);

    // BLUE OBJECTS
    cv::inRange(imgHSV,  cv::Scalar(BLowH, 135, 135), cv::Scalar(BHighH, 255, 255), imgThresholded); //Threshold the image
    //morphological opening (remove small objects from the foreground)
    cv::erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));
    cv::dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(8,8)));

    //morphological closing (fill small holes in the foreground)
    cv::dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(8, 1)) );
    cv::erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(8, 1)) );
    cv::imshow("BLUE OBJECTS", imgThresholded); //show the thresholded image
    cv::waitKey(1);

    // YELLOW OBJECTS
    cv::inRange(imgHSV,  cv::Scalar(YLowH, 160, 160), cv::Scalar(YHighH, 255, 255), imgThresholded); //Threshold the image
    //morphological opening (remove small objects from the foreground)
    cv::erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));
    cv::dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(8,8)));

    //morphological closing (fill small holes in the foreground)
    cv::dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(8, 1)) );
    cv::erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(8, 1)) );
    cv::imshow("YELLOW OBJECTS", imgThresholded); //show the thresholded image
    cv::waitKey(1);

    // VIOLET OBJECTS
    cv::inRange(imgHSV,  cv::Scalar(VLowH, 135, 135), cv::Scalar(VHighH, 255, 255), imgThresholded); //Threshold the image
    //morphological opening (remove small objects from the foreground)
    cv::erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));
    cv::dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(8,8)));

    //morphological closing (fill small holes in the foreground)
    cv::dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(8, 1)) );
    cv::erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(8, 1)) );
    cv::imshow("VIOLET OBJECTS", imgThresholded); //show the thresholded image
    cv::waitKey(1);


}

//    //Invert Image
//    //Go through all the rows
//    for(int i=0; i<cv_ptr->image.rows; i++)
//    {
//        //Go through all the columns
//        for(int j=0; j<cv_ptr->image.cols; j++)
//        {
//            //Go through all the channels (b, g, r)
//            for(int k=0; k<cv_ptr->image.channels(); k++)
//            {
//                //Invert the image by subtracting image data from 255
//                std::cout << cv_ptr->image.data[i*cv_ptr->image.rows*4+j*3 + k] << std::endl;
//                cv_ptr->image.data[i*cv_ptr->image.rows*4+j*3 + k] = 255-cv_ptr->image.data[i*cv_ptr->image.rows*4+j*3 + k];
//            }
//        }
//    }
//    //Display the image using OpenCV
//    cv::imshow(WINDOW, cv_ptr->image);
//    //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
//    cv::waitKey(3);


int main( int argc, char** argv )
{
    ros::init(argc, argv, "colormawsk");
    ros::NodeHandle n;

    image_sub = n.subscribe("/camera/rgb/image_raw", 1, imageCallback);

    ros::Rate loop_rate(1000);

    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
