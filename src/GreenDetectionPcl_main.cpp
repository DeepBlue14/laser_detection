#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
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
    greenDetectionPcl.setActivateGuiBool(config.Activate);
    greenDetectionPcl.setRedInt(config.Red);
    greenDetectionPcl.setGreenInt(config.Green);
    greenDetectionPcl.setBlueInt(config.Blue);
}


int main(int argc, char **argv)
{
    init(argc, argv, "GreenDetectionPcl");


    NodeHandle nh;

    Publisher* mainsPub = greenDetectionPcl.getPublisher();

    Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB>::Ptr >("/camera/depth_registered/points",
                                                                        1,
                                                                        &GreenDetectionPcl::dcallback,
                                                                        &greenDetectionPcl);

    *mainsPub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/frankenscooter/camera/image", 1);

    dynamic_reconfigure::Server<laser_detection::DynConfigConfig> server;
    dynamic_reconfigure::Server<laser_detection::DynConfigConfig>::CallbackType f;
    f = boost::bind(&drCallback, _1, _2);
    server.setCallback(f);


    spin();

    return EXIT_SUCCESS;
}
