/*
 * File:   GreenDetectionPcl_main.cpp
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: 
 *
 * Created May 26, 2015 at 11:00am
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include <dynamic_reconfigure/server.h>
#include <laser_detection/DynConfigConfig.h>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <cstdlib>

#include "GreenDetectionPcl.h"

using namespace ros;
using namespace pcl;
using namespace std;

GreenDetectionPcl greenDetectionPcl;

void drCallback(laser_detection::DynConfigConfig& config, uint32_t level)
{
    if(config.Activate == true)
    {
        greenDetectionPcl.setActivateGuiBool(config.Activate);
        greenDetectionPcl.setRedMinInt(config.Red_Min);
        greenDetectionPcl.setGreenMinInt(config.Green_Min);
        greenDetectionPcl.setBlueMinInt(config.Blue_Min);
        greenDetectionPcl.setRedMaxInt(config.Red_Max);
        greenDetectionPcl.setGreenMaxInt(config.Green_Max);
        greenDetectionPcl.setBlueMaxInt(config.Blue_Max);
    }
}


int main(int argc, char **argv)
{
    init(argc, argv, "GreenDetectionPcl");
    
    ROS_INFO("Starting program\n");

    NodeHandle nh;

    Publisher* mainsPcPubPtr = greenDetectionPcl.getPcPubPtr();
    Publisher* mainsVec3PubPtr = greenDetectionPcl.getVec3PubPtr();

    Subscriber sub = nh.subscribe<PointCloud<PointXYZRGB>::Ptr >("/camera/depth_registered/points",
                                                                        1,
                                                                        &GreenDetectionPcl::dcallback,
                                                                        &greenDetectionPcl);

    *mainsPcPubPtr = nh.advertise<PointCloud<PointXYZRGB> >("/scooter/depth_registered/points", 1);
    *mainsVec3PubPtr = nh.advertise<geometry_msgs::Vector3>("/scooter/vector3/point", 1);

    dynamic_reconfigure::Server<laser_detection::DynConfigConfig> server;
    dynamic_reconfigure::Server<laser_detection::DynConfigConfig>::CallbackType f;
    f = boost::bind(&drCallback, _1, _2);
    server.setCallback(f);


    spin();

    return EXIT_SUCCESS;
}
