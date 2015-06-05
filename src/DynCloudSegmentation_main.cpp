/*
 * File: DynCloudSegmentation_main.cpp
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: 
 *
 * 
 */


#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <laser_detection/CloudParamsConfig.h>

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


void drCallback(laser_detection::CloudParamsConfig& config, uint32_t level)
{
    if(config.Activate == true)
    {
        dynCloudSeg.setActivateGuiBool(config.Activate);
        dynCloudSeg.setInvalid_r(config.Invalid_r);
        dynCloudSeg.setInvalid_g(config.Invalid_g);
        dynCloudSeg.setInvalid_b(config.Invalid_b);
        dynCloudSeg.setInvalid_x(config.Invalid_x);
        dynCloudSeg.setInvalid_y(config.Invalid_y);
        dynCloudSeg.setInvalid_z(config.Invalid_z);
        dynCloudSeg.setLeafSize(config.Leaf_Size);
        dynCloudSeg.setMaxIters(config.Max_Iters);
        dynCloudSeg.setDistThreshold(config.Dist_Thresh);
        dynCloudSeg.setClusterTolerance(config.Cluster_Tol);
        dynCloudSeg.setMinClusterSize(config.Min_clus_pts);
        dynCloudSeg.setMaxClusterSize(config.Max_clus_pts);
    }


}


int main(int argc, char **argv)
{
    init(argc, argv, "DynCloudSegmentation");

    ROS_INFO("Starting node");

    NodeHandle nh;

    Publisher* mainsImagePub = dynCloudSeg.getPublisher();//will this cause an issue, because I am getting 2 pointers to the SAME publisher???
    Publisher* mainsPointcloudPub = dynCloudSeg.getPublisher();

    
    Subscriber imageSub = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color",
                                                             1,
                                                             &DynCloudSegmentation::imageCallback,
                                                             &dynCloudSeg);
    /*
    Subscriber colorSub = nh.subscribe<sensor_msgs::Image>("/scooter/rgb/image_rect_color",
                                                            1,
                                                            &DynCloudSegmentation::imageCallback,
                                                            &dynCloudSeg);
    Subscriber motionSub = nh.subscribe<sensor_msgs::Image>("scooter/rgb/tracked_image",
                                                            1,
                                                            &DynCloudSegmentation::imageCallback,
                                                            &dynCloudSeg);
    Subscriber shapeSub = nh.subscribe<sensor_msgs::Image>("scooter/rgb/shape_image),
                                                            1,
                                                            &DynCloudSegmentation::imageCallback,
                                                            &dynCloudSeg);
    */
    
    Subscriber pointSub = nh.subscribe<geometry_msgs::Point>("/scooter/geometry_msgs/center_point",
                                                              1,
                                                              &DynCloudSegmentation::pointCallback,
                                                              &dynCloudSeg);

    Subscriber pointcloudSub = nh.subscribe<PointCloud<PointXYZRGB>::Ptr>("/camera/depth_registered/points",
                                                                      1,
                                                                      &DynCloudSegmentation::pointcloudCallback,
                                                                      &dynCloudSeg);

    //*mainsImagePub = nh.advertise<sensor_msgs::Image>("/scooter/camera/image", 1);
    *mainsPointcloudPub = nh.advertise<PointCloud<PointXYZRGB> >("/scooter/depth_registered/points", 1);
    
    dynamic_reconfigure::Server<laser_detection::CloudParamsConfig> server;
    dynamic_reconfigure::Server<laser_detection::CloudParamsConfig>::CallbackType f;
    f = boost::bind(&drCallback, _1, _2);
    server.setCallback(f);


    spin();

    return EXIT_SUCCESS;
}
