/*
 * File: ShapeSeg.h
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: This node subscribes to /camera/rgb/image_rect_color, and
 *                   detects circular objects. (...)
 *
 * Reference: https://github.com/Itseez/opencv/blob/master/samples/cpp/squares.cpp
 *
 * Created May 26, 2015 at 6:00pm
 */

#ifndef SHAPE_SEG_H
#define SHAPE_SEG_H

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
//#include <opencv2/imgcodecs/imgcodecs.hpp>
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

class ShapeSeg
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
	    ShapeSeg();
	    void callback(const sensor_msgs::ImageConstPtr& input);
        Mat filterByMotion(Mat nextImage);
        static double angle(Point pt1, Point pt2, Point pt0);
        //void searchForMovement(Mat thresholdImage, Mat& cameraFeed);
	    void setActivateGuiBool(bool activateGuiBool);
	    bool getActivateGuiBool();
        void setSensitivityInt(int sensitivityInt);
        int getSensitivityInt();
        void setBlurInt(int blurInt);
        int getBlurInt();
	    Publisher* getPublisher();
	    ~ShapeSeg();

};

#endif /* SHAPE_SEG_H */
