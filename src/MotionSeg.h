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

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <dynamic_reconfigure/server.h>
#include <laser_detection/ImageParamsConfig.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>


#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <assert.h>

using namespace ros;
using namespace pcl;
using namespace cv;
using namespace std;

class MotionSeg
{
    private:
	    static bool activateGuiBool;
        static int sensitivityInt;
        static int blurInt;
        bool nextIterBool;
        bool objectDetected;
        Mat prevImage;
        Mat hsvImage;
        geometry_msgs::Point centerPoint;
	    Publisher* pub;

    public:
	    MotionSeg();
	    void callback(const sensor_msgs::ImageConstPtr& input);
        void filterByMotion(Mat nextImage);
        void searchForMovement(Mat thresholdImage, Mat& cameraFeed);
        float verifyColor(vector<vector<Point> > movingObjectCoors, Point centerPixel); //also handling size and shape
        bool closeEnough(int x, int y, geometry_msgs::Point theCenterPoint);
        bool hsvExistsNear(Mat cvImage, geometry_msgs::Point centerPoint); //!!!implement!!!
	    void setActivateGuiBool(bool activateGuiBool);
	    bool getActivateGuiBool();
        void setSensitivityInt(int sensitivityInt);
        int getSensitivityInt();
        void setBlurInt(int blurInt);
        int getBlurInt();
        void setCenterPoint(int x, int y);
        geometry_msgs::Point getCenterPoint();
	    Publisher* getPublisher();
	    ~MotionSeg();

};

#endif /* MOTION_SEG_H */
