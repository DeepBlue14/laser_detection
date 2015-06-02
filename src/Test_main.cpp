/*#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <cstdlib>

#include "DynImageSegmentation.h"

using namespace ros;
using namespace pcl;
using namespace std;


DynImageSegmentation dynImageSeg;


int main(int argc, char **argv)
{
    init(argc, argv, "DynImageSegmentation");
    
    ROS_INFO("Starting node\n");

    NodeHandle nh;

    Publisher* mainsPub = dynImageSeg.getPublisher();

    Subscriber sub = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color",
                                                        10,
                                                        &DynImageSegmentation::callback,
                                                        &dynImageSeg);

    *mainsPub = nh.advertise<sensor_msgs::Image>("/scooter/rgb/image", 10);


    spin();
    
    return EXIT_SUCCESS;
}
*/




#include <ros/ros.h>
#include <ros/console.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
//#include <Windows.h>


int main() {
    cv::VideoCapture capWebcam(0);

// get screen resolution, which will be used in future
        //int widthx = GetSystemMetrics(SM_CXSCREEN);
        //int heighty = GetSystemMetrics(SM_CYSCREEN);
        //std::cout << (widthx) << " X " << (heighty) << "\n\n";

//  webcam access
    if (capWebcam.isOpened() == false) {
        std::cout << "Error: Webcam don't work or can't be accessed\n\n";
        return(1);
    }

// creating various necessary staff
    cv::Mat matOriginal;
    cv::Mat matOriginalHSV;
    cv::Mat matProcessed;

    std::vector<cv::Vec3f> vecCircles;
    std::vector<cv::Vec3f>::iterator itrCircles;

    char charEscKey = 0;

    cv::namedWindow("Original", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("Processed", CV_WINDOW_AUTOSIZE);

// creating trackbars
    cv::namedWindow("Trackbar", CV_WINDOW_AUTOSIZE);
    int hmin = 0, smin = 0, vmin = 0,
        hmax = 180, smax = 256, vmax = 256;
    cv::createTrackbar("H min:", "Trackbar", &hmin, hmax);
    cv::createTrackbar("H max:", "Trackbar", &hmax, hmax);
    cv::createTrackbar("S min:", "Trackbar", &smin, smax);
    cv::createTrackbar("S max:", "Trackbar", &smax, smax);
    cv::createTrackbar("V min:", "Trackbar", &vmin, vmax);
    cv::createTrackbar("V max:", "Trackbar", &vmax, vmax);


// main function
    while (charEscKey != 27) {
        if (capWebcam.read(matOriginal) == NULL) {
            std::cout << "Error: frame not readed from Cam\n\n";
            break;
        }

// converting to HSV
        cv::cvtColor(matOriginal, matOriginalHSV, CV_BGR2HSV);

// spliting channels, changing any channel from "matOriginalHSVchannels[0/1/2]" to "matZero" to remove it
        cv::Mat matZero;
        cv::Mat matOriginalHS;
        matZero = cv::Mat::zeros(cv::Size(matOriginalHSV.cols, matOriginalHSV.rows), CV_8UC1);
        cv::vector<cv::Mat> matOriginalHSVchannels(3);
        cv::split(matOriginalHSV, matOriginalHSVchannels);
        cv::vector<cv::Mat> channels;
        channels.push_back(matOriginalHSVchannels[0]);
        channels.push_back(matOriginalHSVchannels[1]);
        channels.push_back(matZero);
        cv::merge(channels, matOriginalHS);

// search of laser spot
        cv::inRange(matOriginalHS, cv::Scalar(hmin, smin, vmin), cv::Scalar(hmax, smax, vmax), matProcessed);

// blur
        cv::GaussianBlur(matProcessed,
            matProcessed,
            cv::Size(5, 5),
            2.2);

// circles search 
        cv::HoughCircles(matProcessed,
            vecCircles,
            CV_HOUGH_GRADIENT,
            1,
            matProcessed.rows / 8,
            100,
            50,
            7,
            20);

// draw circles
        for (itrCircles = vecCircles.begin(); itrCircles != vecCircles.end(); itrCircles++) {
            std::cout << "position x = " << (*itrCircles)[0]
                << ", y = " << (*itrCircles)[1]
                << ", z = " << (*itrCircles)[2] << "\n";

            cv::circle(matOriginalHS,
                cv::Point((int)(*itrCircles)[0], (int)(*itrCircles)[1]),
                1,
                cv::Scalar(255, 0, 0),
                CV_FILLED);

            cv::circle(matOriginalHS,
                cv::Point((int)(*itrCircles)[0], (int)(*itrCircles)[1]),
                (int)(*itrCircles)[2],
                cv::Scalar(255, 255, 0),
                2);

// set mouse cursor on laser coordinates
            int screenx = (int)((*itrCircles)[0] * 2.2);
            int screeny = (int)((*itrCircles)[1] * 1.2);
            //SetCursorPos(screenx, screeny);

        }
        
        cv::Mat matProcessedInv;
        cv::bitwise_not(matProcessed, matProcessedInv);
        
// show windows
        cv::imshow("Original", matOriginalHS);
        //cv::imshow("Processed", matProcessed);
        cv::imshow("Processed", matProcessedInv);
        charEscKey = cv::waitKey(10);
    }
    return(0);
}

