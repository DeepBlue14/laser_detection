/*
 * File: DynImageSegmentation_main.cpp
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: 
 *
 * 
 */
 
 
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <laser_detection/ImageParamsConfig.h>

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


void drCallback(laser_detection::ImageParamsConfig& config, uint32_t level)
{
    if(config.Activate == true)
    {
        dynImageSeg.setActivateGuiBool(config.Activate);
        dynImageSeg.setRedMinInt(config.Red_Min);
        dynImageSeg.setGreenMinInt(config.Green_Min);
        dynImageSeg.setBlueMinInt(config.Blue_Min);
        dynImageSeg.setRedMaxInt(config.Red_Max);
        dynImageSeg.setGreenMaxInt(config.Green_Max);
        dynImageSeg.setBlueMaxInt(config.Blue_Max);

        dynImageSeg.setSensitivityInt(config.Sensitivity);
        dynImageSeg.setBlurInt(config.Blur);
    }
    else
    {
        ROS_INFO("GUI has not been activated\n");
    }
}


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

    *mainsPub = nh.advertise<sensor_msgs::Image>("/scooter/camera/image", 10);

    dynamic_reconfigure::Server<laser_detection::ImageParamsConfig> server;
    dynamic_reconfigure::Server<laser_detection::ImageParamsConfig>::CallbackType f;
    f = boost::bind(&drCallback, _1, _2);
    server.setCallback(f);


    spin();
    
    return EXIT_SUCCESS;
}
