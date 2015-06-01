/*
 * File: GreenDetectionCv.h
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: 
 *
 * Reference: http://docs.opencv.org/doc/tutorials/imgproc/shapedescriptors/bounding_rects_circles/bounding_rects_circles.html
 *
 * Created May 26, 2015 at 6:00pm
 */

#ifndef DYN_IMAGE_SEGMENTATION_H
#define DYN_IMAGE_SEGMENTATION_H

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <laser_detection/ColorParamsConfig.h>

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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include <iostream>
#include <vector>
#include <assert.h>

using namespace ros;
using namespace pcl;
using namespace cv;
using namespace std;

class DynImageSegmentation
{
    private:
	    static bool activateGuiBool;
	    static int redMinInt;
	    static int greenMinInt;
	    static int blueMinInt;
        static int redMaxInt;
        static int greenMaxInt;
        static int blueMaxInt;

        vector<int> validXVec;
        vector<int> validYVec;
        int minX;
        int minY;
        int maxX;
        int maxY;

	    Publisher* pub;

    public:
	    DynImageSegmentation();
	    void callback(const sensor_msgs::ImageConstPtr& input);
        //void filterByColor();
        //void drawBoundingBox();
	    void setActivateGuiBool(bool activateGuiBool);
	    bool getActivateGuiBool();
	    void setRedMinInt(int redMinInt);
	    int getRedMinInt();
	    void setGreenMinInt(int greenMinInt);
	    int getGreenMinInt();
	    void setBlueMinInt(int blueMinInt);
	    int getBlueMinInt();
        void setRedMaxInt(int redMaxInt);
        int getRedMaxInt();
        void setGreenMaxInt(int greenMaxInt);
        int getGreenMaxInt();
        void setBlueMaxInt(int blueMaxInt);
        int getBlueMaxInt();
	    Publisher* getPublisher();
	    ~DynImageSegmentation();

};

#endif /* LASER_DETECTION_CV_H */
