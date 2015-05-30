#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
//dyn config

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <cstdlib>

#include "GreenPart2.h"

using namespace ros;
using namespace pcl;
using namespace std;

GreenPart2 greenPart2;

int main(int argc, char **argv)
{
    init(argc, argv, "GreenPart2");

    ROS_INFO("Starting node");

    NodeHandle nh;

    Publisher* mainsImagePub = greenPart2.getPublisher();//will this cause an issue, because I am getting 2 pointers to the SAME publisher???
    Publisher* mainsPointcloudPub = greenPart2.getPublisher();

    /*
    Subscriber imageSub = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color",
                                                             1,
                                                             &GreenPart2::imageCallback,
                                                             &greenPart2);
    */
    Subscriber imageSub = nh.subscribe<sensor_msgs::Image>("/scooter/camera/image",
                                                            1,
                                                            &GreenPart2::imageCallback,
                                                            &greenPart2);

    Subscriber pointcloudSub = nh.subscribe<PointCloud<PointXYZRGB>::Ptr>("/camera/depth_registered/points",
                                                                      1,
                                                                      &GreenPart2::pointcloudCallback,
                                                                      &greenPart2);

    //*mainsImagePub = nh.advertise<sensor_msgs::Image>("/scooter/camera/image", 1);
    *mainsPointcloudPub = nh.advertise<PointCloud<PointXYZRGB> >("/scooter/depth_registered/points", 1);

    spin();

    return EXIT_SUCCESS;
}
