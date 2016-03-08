/*
 * File: MotionSeg_main.cpp
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
//#include <laser_detection/ImageParamsConfig.h>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream> 
#include <cstdlib>

#include "MotionSeg.h"

using namespace ros;
using namespace pcl;
using namespace std;





/*void drCallback(laser_detection::ImageParamsConfig& config, uint32_t level)
{
    if(config.Activate == true)
    {
        motionSeg.setActivateGuiBool(config.Activate);

        motionSeg.setSensitivityInt(config.Motion_Sensitivity);
        motionSeg.setBlurInt(config.Motion_Blur);
    }
    else
    {
        ROS_INFO("GUI has not been activated\n");
    }
}*/


int main(int argc, char **argv)
{
    init(argc, argv, "motionSegmentation");
    
    ROS_INFO("Starting node\n");

    MotionSeg motionSeg;
    NodeHandle nh;
    Publisher* mainsPub = motionSeg.getPublisher();
    Subscriber sub = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color",
                                                        10,
                                                        &MotionSeg::callback,
                                                        &motionSeg);

    //*mainsPub = nh.advertise<geometry_msgs::Point>("/scooter/geometry_msgs/center_point", 10);
    *mainsPub = nh.advertise<geometry_msgs::Point>("tablet/geometry_msgs/point", 10);

    //dynamic_reconfigure::Server<laser_detection::ImageParamsConfig> server;
    //dynamic_reconfigure::Server<laser_detection::ImageParamsConfig>::CallbackType f;
    //f = boost::bind(&drCallback, _1, _2);
    //server.setCallback(f);


    spin();
    
    return EXIT_SUCCESS;
}
