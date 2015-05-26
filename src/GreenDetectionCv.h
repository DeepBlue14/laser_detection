#ifndef LASER_DETECTION_CV_H
#define LASER_DETECTION_CV_H

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <laser_detection/DynConfigConfig.h>

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

class GreenDetectionCv
{
    private:
	static bool activateGuiBool;
	static int redInt;
	static int greenInt;
	static int blueInt;

	Publisher* pub;

    public:
	GreenDetectionCv();
	void callback(const sensor_msgs::ImageConstPtr& input);
    //void filterByColor();
	void setActivateGuiBool(bool activateGuiBool);
	bool getActivateGuiBool();
	void setRedInt(int redInt);
	int getRedInt();
	void setGreenInt(int greenInt);
	int getGreenInt();
	void setBlueInt(int blueInt);
	int getBlueInt();
	Publisher* getPublisher();
	~GreenDetectionCv();

};

#endif /* LASER_DETECTION_CV_H */
