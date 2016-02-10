/*
 * File: EvalPoint_main.cpp
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
//#include <laser_detection/EvalPointConfig.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <cstdlib>

#include "EvalPoint.h"

using namespace ros;
using namespace pcl;
using namespace std;

EvalPoint evalPoint;


///*
void drCallback(laser_detection::EvalPointConfig& config, uint32_t level)
{
    if(config.Activate == true)
    {
        evalPoint.setActivateGuiBool(config.Activate);
        evalPoint.setInvalid_r(config.Invalid_r);
        evalPoint.setInvalid_g(config.Invalid_g);
        evalPoint.setInvalid_b(config.Invalid_b);
        evalPoint.setInvalid_x(config.Invalid_x);
        evalPoint.setInvalid_y(config.Invalid_y);
        evalPoint.setInvalid_z(config.Invalid_z);
        evalPoint.setLeafSize(config.Leaf_Size);
        evalPoint.setMaxIters(config.Max_Iters);
        evalPoint.setDistThreshold(config.Dist_Thresh);
        evalPoint.setClusterTolerance(config.Cluster_Tol);
        evalPoint.setMinClusterSize(config.Min_clus_pts);
        evalPoint.setMaxClusterSize(config.Max_clus_pts);
    }


}
//*/


int main(int argc, char **argv)
{
    init(argc, argv, "EvalPoint");

    ROS_INFO("Starting node");

    NodeHandle nh;

    Publisher* mainsImagePub = evalPoint.getPublisher();//will this cause an issue, because I am getting 2 pointers to the SAME publisher???
    Publisher* mainsPointcloudPub = evalPoint.getPublisher();

    
    Subscriber imageSub = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color",
                                                             1,
                                                             &EvalPoint::imageCallback,
                                                             &evalPoint);

    
    Subscriber pointSub = nh.subscribe<geometry_msgs::Point>("/scooter/geometry_msgs/center_point",
                                                              1,
                                                              &EvalPoint::pointCallback,
                                                              &evalPoint);

    Subscriber pointcloudSub = nh.subscribe<PointCloud<PointXYZRGB>::Ptr>("/camera/depth_registered/points",
                                                                      1,
                                                                      &EvalPoint::pointcloudCallback,
                                                                      &evalPoint);

    //*mainsImagePub = nh.advertise<sensor_msgs::Image>("/scooter/camera/image", 1);
    *mainsPointcloudPub = nh.advertise<PointCloud<PointXYZRGB> >("/scooter/rgb/image_with_box", 1);
    
    ///*
    dynamic_reconfigure::Server<laser_detection::EvalPointConfig> server;
    dynamic_reconfigure::Server<laser_detection::EvalPointConfig>::CallbackType f;
    f = boost::bind(&drCallback, _1, _2);
    server.setCallback(f);
    //*/

    spin();

    return EXIT_SUCCESS;
}
