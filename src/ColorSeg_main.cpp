/*
 * File: ColorSeg_main.cpp
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

#include "ColorSeg.h"

using namespace ros;
using namespace pcl;
using namespace std;


ColorSeg colorSeg;


void drCallback(laser_detection::ImageParamsConfig& config, uint32_t level)
{
    if(config.Activate == true)
    {
        colorSeg.setActivateGuiBool(config.Activate);
        colorSeg.setRedMinInt(config.Red_Min);
        colorSeg.setGreenMinInt(config.Green_Min);
        colorSeg.setBlueMinInt(config.Blue_Min);
        colorSeg.setRedMaxInt(config.Red_Max);
        colorSeg.setGreenMaxInt(config.Green_Max);
        colorSeg.setBlueMaxInt(config.Blue_Max);
        
        colorSeg.setHMinInt(config.H_Min);
        colorSeg.setSMinInt(config.S_Min);
        colorSeg.setVMinInt(config.V_Min);
        colorSeg.setHMaxInt(config.H_Max);
        colorSeg.setSMaxInt(config.S_Max);
        colorSeg.setVMaxInt(config.V_Max);
    }
    else
    {
        ROS_INFO("GUI has not been activated\n");
    }
}


int main(int argc, char **argv)
{
    init(argc, argv, "colorSegmentation");
    
    ROS_INFO("Starting node\n");

    NodeHandle nh;

    Publisher* mainsPub = colorSeg.getPublisher();

    Subscriber sub = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color",
                                                        10,
                                                        &ColorSeg::callback,
                                                        &colorSeg);

    *mainsPub = nh.advertise<sensor_msgs::Image>("/scooter/rgb/image_rect_color", 10);

    dynamic_reconfigure::Server<laser_detection::ImageParamsConfig> server;
    dynamic_reconfigure::Server<laser_detection::ImageParamsConfig>::CallbackType f;
    f = boost::bind(&drCallback, _1, _2);
    server.setCallback(f);


    spin();
    
    return EXIT_SUCCESS;
}
