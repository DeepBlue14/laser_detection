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

#include "DynCloudSegmentation.h"

using namespace ros;
using namespace pcl;
using namespace std;

DynCloudSegmentation dynCloudSeg;

int main(int argc, char **argv)
{
    init(argc, argv, "DynCloudSegmentation");

    ROS_INFO("Starting node");

    NodeHandle nh;

    Publisher* mainsImagePub = dynCloudSeg.getPublisher();//will this cause an issue, because I am getting 2 pointers to the SAME publisher???
    Publisher* mainsPointcloudPub = dynCloudSeg.getPublisher();

    /*
    Subscriber imageSub = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color",
                                                             1,
                                                             &DynCloudSegmentation::imageCallback,
                                                             &DynCloudSegmentation);
    */
    Subscriber imageSub = nh.subscribe<sensor_msgs::Image>("/scooter/camera/image",
                                                            1,
                                                            &DynCloudSegmentation::imageCallback,
                                                            &dynCloudSeg);

    Subscriber pointcloudSub = nh.subscribe<PointCloud<PointXYZRGB>::Ptr>("/camera/depth_registered/points",
                                                                      1,
                                                                      &DynCloudSegmentation::pointcloudCallback,
                                                                      &dynCloudSeg);

    //*mainsImagePub = nh.advertise<sensor_msgs::Image>("/scooter/camera/image", 1);
    *mainsPointcloudPub = nh.advertise<PointCloud<PointXYZRGB> >("/scooter/depth_registered/points", 1);

    spin();

    return EXIT_SUCCESS;
}
