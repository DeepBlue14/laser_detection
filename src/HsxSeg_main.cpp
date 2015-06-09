/*
 * File: HsxSeg_main.cpp
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
#include <laser_detection/HsxParamsConfig.h>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <cstdlib>

#include "HsxSeg.h"

using namespace ros;
using namespace pcl;
using namespace std;


HsxSeg hsxSeg;


void drCallback(laser_detection::HsxParamsConfig& config, uint32_t level)
{
    if(config.Activate == true)
    {
        hsxSeg.setActivateGuiBool(config.Activate);
        hsxSeg.setHMinDbl(config.H_Min);
        hsxSeg.setSMinDbl(config.S_Min);
        hsxSeg.setVMinDbl(config.V_Min);
        hsxSeg.setHMaxDbl(config.H_Max);
        hsxSeg.setSMaxDbl(config.S_Max);
        hsxSeg.setVMaxDbl(config.V_Max);
        hsxSeg.setSensitivityInt(config.Sensitivity);
        hsxSeg.setBlurInt(config.Blur);
    }
    else
    {
        ROS_INFO("GUI has not been activated");
    }
}


int main(int argc, char **argv)
{
    init(argc, argv, "HsxSegmentation");
    
    ROS_INFO("Starting node\n");

    NodeHandle nh;

    Publisher* mainsPub = hsxSeg.getPublisher();

    Subscriber sub = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color",
                                                        10,
                                                        &HsxSeg::callback,
                                                        &hsxSeg);

    *mainsPub = nh.advertise<sensor_msgs::Image>("/scooter/rgb/image_rect_hsx", 10);

    dynamic_reconfigure::Server<laser_detection::HsxParamsConfig> server;
    dynamic_reconfigure::Server<laser_detection::HsxParamsConfig>::CallbackType f;
    server.setCallback(f);


    spin();
    
    return EXIT_SUCCESS;
}
