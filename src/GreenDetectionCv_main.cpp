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

#include "GreenDetectionCv.h"

using namespace ros;
using namespace pcl;
using namespace std;


GreenDetectionCv greenDetectionCv;


void drCallback(laser_detection::DynConfigConfig& config, uint32_t level)
{
    if(config.Activate == true)
    {
        greenDetectionCv.setActivateGuiBool(config.Activate);
        greenDetectionCv.setRedInt(config.Red);
        greenDetectionCv.setGreenInt(config.Green);
        greenDetectionCv.setBlueInt(config.Blue);
    }
}


int main(int argc, char **argv)
{
    init(argc, argv, "GreenDetectionCv");


    NodeHandle nh;

    Publisher* mainsPub = greenDetectionCv.getPublisher();

    Subscriber sub = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color",
                                                        10,
                                                        &GreenDetectionCv::callback,
                                                        &greenDetectionCv);

    *mainsPub = nh.advertise<sensor_msgs::Image>("/frankenscooter/camera/image", 10);

    dynamic_reconfigure::Server<laser_detection::DynConfigConfig> server;
    dynamic_reconfigure::Server<laser_detection::DynConfigConfig>::CallbackType f;
    f = boost::bind(&drCallback, _1, _2);
    server.setCallback(f);


    spin();

    return EXIT_SUCCESS;
}
