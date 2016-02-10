/*
 * File: ColorSeg.h
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: This node subscribes to /camera/rgb/image_rect_color, and 
 *                   dynamically allows the user to filter the image by rgb
 *                   values.
 *
 *
 * Created May 26, 2015 at 6:00pm
 */

#ifndef COLOR_SEG_H
#define COLOR_SEG_H

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
//#include <dynamic_reconfigure/server.h>
//#include <laser_detection/ImageParamsConfig.h>

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

class ColorSeg
{
    private:
	    static bool activateGuiBool;
	    
	    static int redMinInt;
	    static int greenMinInt;
	    static int blueMinInt;
        static int redMaxInt;
        static int greenMaxInt;
        static int blueMaxInt;

        static int hMinInt;
        static int sMinInt;
        static int vMinInt;
        static int hMaxInt;
        static int sMaxInt;
        static int vMaxInt;

        vector<int> validXVec;
        vector<int> validYVec;
        int minX;
        int minY;
        int maxX;
        int maxY;

	    Publisher* pub;

    public:
	    ColorSeg();
	    void callback(const sensor_msgs::ImageConstPtr& input);
	    
	    void setActivateGuiBool(bool activateGuiBool);
	    bool getActivateGuiBool();
	    
	    //RGB
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
        
        //HSV
        void setHMinInt(int hMinInt);
        int getHMinInt();
        void setSMinInt(int sMinInt);
        int getSMinInt();
        void setVMinInt(int vMinInt);
        int getVMinInt();
        void setHMaxInt(int hMaxInt);
        int getHMaxInt();
        void setSMaxInt(int sMaxInt);
        int getSMaxInt();
        void setVMaxInt(int vMaxInt);
        int getVMaxInt();
        
	    Publisher* getPublisher();
	    ~ColorSeg();

};

#endif /* COLOR_SEG_H */
