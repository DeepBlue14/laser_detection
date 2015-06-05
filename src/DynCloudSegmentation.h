/*
 * File: DynCloudSegmentation.h
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: 
 *
 * 
 */
//fun: http://pointclouds.org/documentation/tutorials/pairwise_incremental_registration.php

#ifndef DYN_CLOUD_SEGMENTATION_H
#define DYN_CLOUD_SEGMENTATION_H

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <dynamic_reconfigure/server.h>
#include <laser_detection/CloudParamsConfig.h>

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

class DynCloudSegmentation
{
    private:
        sensor_msgs::ImageConstPtr image;
        geometry_msgs::PointConstPtr pixelPoint;
        geometry_msgs::Point realWorldCoorPoint;
        cv::Mat m_image;
        PointCloud<PointXYZRGB>::Ptr cloud;
        Publisher* pub;
        bool imageIsReady;
        bool pointIsReady;
        bool cloudIsReady;

        static bool activateGuiBool;
        static int invalid_r;
        static int invalid_g;
        static int invalid_b;
        static double invalid_x;
        static double invalid_y;
        static double invalid_z;
        static double leafSize;
        static int maxIters;
        static double distThreshold;
        static double clusterTolerance;
        static int minClusterSize;
        static int maxClusterSize;

        vector<geometry_msgs::Point> clusterCenterVec;

    public:
        DynCloudSegmentation();
        void imageCallback(const sensor_msgs::ImageConstPtr& image);
        void pointCallback(const geometry_msgs::PointConstPtr& pixelPoint);
        void pointcloudCallback(PointCloud<PointXYZRGB>::Ptr cloud);
        void copyColorToCloud();
        PointCloud<PointXYZRGB>::Ptr euclideanClusterExtraction(PointCloud<PointXYZRGB>::Ptr cloud);
        void conditionalEuclideanClustering(PointCloud<PointXYZRGB>::Ptr cloud);
        Publisher* getPublisher();
        
       void setActivateGuiBool(bool activateGuiBool);
       bool getActivateGuiBool();
       void setInvalid_r(int invalid_r);
       int getInvalid_r();
       void setInvalid_g(int invalid_g);
       int getInvalid_g();
       void setInvalid_b(int invalid_b);
       int getInvalid_b();
       void setInvalid_x(double invalid_x);
       double getInvalid_x();
       void setInvalid_y(double invalid_y);
       double getInvalid_y();
       void setInvalid_z(double invalid_z);
       double getInvalid_z();
       void setLeafSize(double leafSize);
       double getLeafSize();
       void setMaxIters(int maxIters);
       int getMaxIters();
       void setDistThreshold(double distThreshold);
       double getDistThreshold();
       void setClusterTolerance(double clusterTolerance);
       double getClusterTolerance();
       void setMinClusterSize(int minClusterSize);
       int getMinClusterSize();
       void setMaxClusterSize(int maxClusterSize);
       int getMaxClusterSize();
        ~DynCloudSegmentation();
};

#endif /* DYN_CLOUD_SEGMENTATION_H */
