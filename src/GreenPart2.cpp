#include "GreenPart2.h"


GreenPart2::GreenPart2()
{
    pub = new Publisher();
}


void GreenPart2::imageCallback(const sensor_msgs::ImageConstPtr& image)
{
    this->image = image;
}


void GreenPart2::pointcloudCallback(PointCloud<PointXYZRGB>::Ptr cloud)
{
    this->cloud = cloud;

    copyColorToCloud();
}


void GreenPart2::copyColorToCloud()
{
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat cvImage;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what() );
    }

    cvImage = cv_ptr->image;


    int pclCount = 0;
    for(size_t y = 0; y < cvImage.rows; y++)
    {//ROS_INFO("Beginning outer loop");
        for(size_t x = 0; x < cvImage.cols; x++)
        {
            Vec3b color = cvImage.at<Vec3b>(Point(x,y));
            cloud->points[pclCount].r = color.val[2];
            cloud->points[pclCount].g = color.val[1];
            cloud->points[pclCount].b = color.val[0];
            
            // Set xyz fields of blacked out points 
            
            /*
            cloud->points[pclCount].r = 0;
            cloud->points[pclCount].g = 0;
            cloud->points[pclCount].b = 0;
           */ 

            //cout << "(pre) pixel z: " << cloud->points[pclCount].z << endl;
            if( !(cloud->points[pclCount].z >= 0) && !(cloud->points[pclCount].z <= 255) )
            {
                //cout << "Found NaN!" << endl;
                cloud->points[pclCount].x = 1;
                cloud->points[pclCount].y = 1;
                cloud->points[pclCount].z = 1;
                cloud->points[pclCount].r = 255;
                cloud->points[pclCount].g = 0;
                cloud->points[pclCount].b = 0;
            }
            else if(pclCount < 10)
            {
                //cout << "valid" << endl;
            }

            //cout << "(post) pixel z: " << cloud->points[pclCount].z << endl;
            pclCount++;
        } // end of outer for loop
    }

    cloud = euclideanClusterExtraction(cloud);

    pub->publish(*cloud);
}


PointCloud<PointXYZRGB>::Ptr GreenPart2::euclideanClusterExtraction(PointCloud<PointXYZRGB>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    int i=0, nr_points = (int) cloud_filtered->points.size ();
    while (cloud_filtered->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! CHANGE THIS PARAM !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    /*int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]); 
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        //std::stringstream ss;
        //ss << "cloud_cluster_" << j << ".pcd";
        //writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false);
        j++;
    }
    */

    return cloud_filtered;
}


void GreenPart2::conditionalEuclideanClustering(PointCloud<PointXYZRGB>::Ptr cloud)
{
    ;
}


Publisher* GreenPart2::getPublisher()
{
    return pub;
}


GreenPart2::~GreenPart2()
{
    ;
}
