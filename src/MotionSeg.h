/*
 * File: MotionSeg.h
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: This node subscribes to /camera/rgb/image_rect_color, and
 *                   detects and tracks the largest moving object.  The user
 *                   may dynamically reconfigure the sensitivity and blur values
 *                   via ROS's rqt_reconfigure GUI.
 *
 * Reference: http://docs.opencv.org/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html
 *
 * Created May 26, 2015 at 6:00pm
 */
 
#ifndef MOTION_SEG_H
#define MOTION_SEG_H

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>

// OpenCV
#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//STL
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <vector>

using namespace ros;
using namespace cv;
using namespace std;

class MotionSeg
{
    private:
        bool isInitialized;
        const int SENSITIVITY;
        const int BLUR;
        bool nextIterBool;
        bool objectDetected;
        Mat prevImage;
        Mat hsvImage;
        geometry_msgs::Point centerPoint;
	    Publisher* pub;

    public:
	    MotionSeg(int sensitivity = 20, int blue = 25);
	    void callback(const sensor_msgs::ImageConstPtr& input);
        void filter(Mat nextImage);
        void searchForMovement(Mat thresholdImage, Mat& cameraFeed);
        bool verifySize();
        float verifyColor(vector<vector<Point> > movingObjectCoors, Point centerPixel); //also handling size and shape
        bool closeEnough(int x, int y, geometry_msgs::Point theCenterPoint);
        bool hsvExistsNear(Mat cvImage, geometry_msgs::Point centerPoint); //!!!implement!!!

        void setCenterPoint(int x, int y);
        geometry_msgs::Point getCenterPoint();
	    Publisher* getPublisher();
	    
	    string* toString();
	    ~MotionSeg();

};

#endif /* MOTION_SEG_H */
