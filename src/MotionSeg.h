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

        vector<int> validXVec;
        vector<int> validYVec;
        int minX;
        int minY;
        int maxX;
        int maxY;
        
        bool nextIterBool;
        Mat prevImage;

	    Publisher* pub;

    public:
	    MotionSeg();
	    void callback(const sensor_msgs::ImageConstPtr& input);
        Mat filterByMotion(Mat nextImage);
        void searchForMovement(Mat thresholdImage, Mat& cameraFeed);
	    void setActivateGuiBool(bool activateGuiBool);
	    bool getActivateGuiBool();
        void setSensitivityInt(int sensitivityInt);
        int getSensitivityInt();
        void setBlurInt(int blurInt);
        int getBlurInt();
	    Publisher* getPublisher();
	    ~MotionSeg();

};

#endif /* MOTION_SEG_H */
