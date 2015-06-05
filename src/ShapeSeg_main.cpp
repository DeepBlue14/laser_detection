/*
 * File: shapeSeg_main.cpp
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

#include "ShapeSeg.h"

using namespace ros;
using namespace pcl;
using namespace std;


ShapeSeg shapeSeg;


void drCallback(laser_detection::ImageParamsConfig& config, uint32_t level)
{
    if(config.Activate == true)
    {
        shapeSeg.setActivateGuiBool(config.Activate);

        shapeSeg.setSensitivityInt(config.Shape_Sensitivity);
        shapeSeg.setBlurInt(config.Shape_Blur);
    }
    else
    {
        ROS_INFO("GUI has not been activated\n");
    }
}


int main(int argc, char **argv)
{
    init(argc, argv, "shapeSegmentation");
    
    ROS_INFO("Starting node\n");

    NodeHandle nh;

    Publisher* mainsPub = shapeSeg.getPublisher();

    Subscriber sub = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color",
                                                        10,
                                                        &ShapeSeg::callback,
                                                        &shapeSeg);

    *mainsPub = nh.advertise<sensor_msgs::Image>("/scooter/rgb/shape_image", 10);

    dynamic_reconfigure::Server<laser_detection::ImageParamsConfig> server;
    dynamic_reconfigure::Server<laser_detection::ImageParamsConfig>::CallbackType f;
    f = boost::bind(&drCallback, _1, _2);
    server.setCallback(f);


    spin();
    
    return EXIT_SUCCESS;
}
