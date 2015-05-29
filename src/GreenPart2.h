//fun: http://pointclouds.org/documentation/tutorials/pairwise_incremental_registration.php

#ifndef GREEN_PART2_H
#define GREEN_PART2_H

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
//include dyn reconfig

#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/console/time.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include <iostream>
#include <vector>

using namespace ros;
using namespace cv;
using namespace pcl;
using namespace std;

class GreenPart2
{
    private:
        sensor_msgs::ImageConstPtr image;
        PointCloud<PointXYZRGB>::Ptr cloud;
        Publisher* pub;

    public:
        GreenPart2();
        void imageCallback(const sensor_msgs::ImageConstPtr& image);
        void pointcloudCallback(PointCloud<PointXYZRGB>::Ptr cloud);
        void copyColorToCloud();
        Publisher* getPublisher();
        ~GreenPart2();
};

#endif /* GREEN_PART2_H */
