/*
 * File: HsxSeg.h
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: This node subscribes to /camera/rgb/image_rect_color,
 *                   converts it to a HXL or HSV (change it in the source file)
 *                   and publishes it.
 *
 * Reference: http://answers.opencv.org/question/52754/laser-pointer-detect-and-track/
 *
 * Created May 26, 2015 at 6:00pm
 */

#ifndef HSX_SEG_H
#define HSX_SEG_H

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

class HsxSeg
{
    private:
	    Publisher* pub;

    public:
	    HsxSeg();
	    void callback(const sensor_msgs::ImageConstPtr& input);
	    Publisher* getPublisher();
	    ~HsxSeg();

};

#endif /* HSX_SEG_H */
