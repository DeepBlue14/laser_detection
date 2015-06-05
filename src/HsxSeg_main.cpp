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


    spin();
    
    return EXIT_SUCCESS;
}
