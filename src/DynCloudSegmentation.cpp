#include "DynCloudSegmentation.h"


bool DynCloudSegmentation::activateGuiBool = false;
int DynCloudSegmentation::invalid_r = 255;
int DynCloudSegmentation::invalid_g = 0;
int DynCloudSegmentation::invalid_b = 0;
double DynCloudSegmentation::invalid_x = 1;
double DynCloudSegmentation::invalid_y = 1;
double DynCloudSegmentation::invalid_z = 1;
double DynCloudSegmentation::leafSize = 0.01;
int DynCloudSegmentation::maxIters = 100;
double DynCloudSegmentation::distThreshold = 0.02;
double DynCloudSegmentation::clusterTolerance = 0.02;
int DynCloudSegmentation::minClusterSize = 100;
int DynCloudSegmentation::maxClusterSize = 25000;


DynCloudSegmentation::DynCloudSegmentation()
{
    pub = new Publisher();
    readyBool = false;
}


void DynCloudSegmentation::imageCallback(const sensor_msgs::ImageConstPtr& image)
{
    this->image = image;
    readyBool = true;
}


void DynCloudSegmentation::pointcloudCallback(PointCloud<PointXYZRGB>::Ptr cloud)
{
    this->cloud = cloud;

    //makesure that the image has been initialized; without this check
    //there would be a seg fault.
    if(readyBool == true)
    {
        copyColorToCloud();
    }
}


void DynCloudSegmentation::copyColorToCloud()
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
            //if( !(cloud->points[pclCount].z >= 0) && !(cloud->points[pclCount].z <= 255) )
           // {
                //cout << "Found NaN!" << endl;
                /*
                cloud->points[pclCount].x = 1;
                cloud->points[pclCount].y = 1;
                cloud->points[pclCount].z = 1;
                cloud->points[pclCount].r = 255;
                cloud->points[pclCount].g = 0;
                cloud->points[pclCount].b = 0;
                */
                //cloud->points[pclCount].x = getInvalid_x();
                //cloud->points[pclCount].y = getInvalid_y();
               /* cloud->points[pclCount].z = getInvalid_z();
                cloud->points[pclCount].r = getInvalid_r();
                cloud->points[pclCount].g = getInvalid_g();
                cloud->points[pclCount].b = getInvalid_b();
            }
            else if( (cloud->points[pclCount].r != 0) || (cloud->points[pclCount].g != 0) || (cloud->points[pclCount].b != 0) )
            {
                cloud->points[pclCount].z = getInvalid_z();
            }*/

            //cout << "(post) pixel z: " << cloud->points[pclCount].z << endl;
            pclCount++;
        } // end of outer for loop
    }

     cloud = euclideanClusterExtraction(cloud);

    pub->publish(*cloud);
}


PointCloud<PointXYZRGB>::Ptr DynCloudSegmentation::euclideanClusterExtraction(PointCloud<PointXYZRGB>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    vg.setInputCloud (cloud);
    //vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.setLeafSize (getLeafSize(), getLeafSize(), getLeafSize() );
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
    //seg.setMaxIterations (100);
    seg.setMaxIterations(getMaxIters() );
    //seg.setDistanceThreshold (0.02);
    seg.setDistanceThreshold(getDistThreshold() );

    int i=0, nr_points = (int) cloud_filtered->points.size ();
    while (cloud_filtered->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            //std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

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
    //ec.setClusterTolerance (0.02); // 2cm
    ec.setClusterTolerance(getClusterTolerance() );
    //ec.setMinClusterSize (100);
    ec.setMinClusterSize(getMinClusterSize() );
    //ec.setMaxClusterSize (25000);
    ec.setMaxClusterSize(getMaxClusterSize() );
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    int j = 0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloud_cluster = *cloud;
   
    for(size_t i = 0; i < cloud_cluster->points.size(); i++)
    {
        cloud_cluster->points[i].r = getInvalid_r();
        cloud_cluster->points[i].g = getInvalid_g();
        cloud_cluster->points[i].b = getInvalid_b();
    }

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);//old
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            cloud_cluster->points[*pit] = cloud_filtered->points[*pit];
            //cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
            cloud_cluster->points[*pit].r = 0;
            cloud_cluster->points[*pit].g = 0;
            cloud_cluster->points[*pit].b = 175;
            //cout << "updated color" << *pit << endl;
        }

        j++;
    }
    
    cout << "Number of clusters: " << cluster_indices.size() << endl;

    //return cloud_filtered;
    return cloud_cluster;
    //return cloud;
}


void DynCloudSegmentation::conditionalEuclideanClustering(PointCloud<PointXYZRGB>::Ptr cloud)
{
    ;
}


void DynCloudSegmentation::setActivateGuiBool(bool activateGuiBool)
{
    this->activateGuiBool = activateGuiBool;
}


bool DynCloudSegmentation::getActivateGuiBool()
{
    return activateGuiBool;
}


void DynCloudSegmentation::setInvalid_r(int invalid_r)
{
    this->invalid_r = invalid_r;
}


int DynCloudSegmentation::getInvalid_r()
{
    return invalid_r;
}


void DynCloudSegmentation::setInvalid_g(int invalid_g)
{
    this->invalid_g = invalid_g;
}


int DynCloudSegmentation::getInvalid_g()
{
    return invalid_g;
}


void DynCloudSegmentation::setInvalid_b(int invalid_b)
{
    this->invalid_b = invalid_b;
}


int DynCloudSegmentation::getInvalid_b()
{
    return invalid_b;
}


void DynCloudSegmentation::setInvalid_x(double invalid_x)
{
    this->invalid_x = invalid_x;
}


double DynCloudSegmentation::getInvalid_x()
{
    return invalid_x;
}


void DynCloudSegmentation::setInvalid_y(double invalid_y)
{
    this->invalid_y = invalid_y;
}


double DynCloudSegmentation::getInvalid_y()
{
    return invalid_y;
}


void DynCloudSegmentation::setInvalid_z(double invalid_z)
{
    this->invalid_z = invalid_z;
}


double DynCloudSegmentation::getInvalid_z()
{
    return invalid_z;
}


void DynCloudSegmentation::setLeafSize(double leafSize)
{
    this->leafSize = leafSize;
}


double DynCloudSegmentation::getLeafSize()
{
    return leafSize;
}


void DynCloudSegmentation::setMaxIters(int maxIters)
{
    this->maxIters = maxIters;
}


int DynCloudSegmentation::getMaxIters()
{
    return maxIters;
}


void DynCloudSegmentation::setDistThreshold(double distThreshold)
{
    this->distThreshold = distThreshold;
}


double DynCloudSegmentation::getDistThreshold()
{
    return distThreshold;
}


void DynCloudSegmentation::setClusterTolerance(double clusterTolerance)
{
    this->clusterTolerance = clusterTolerance;
}


double DynCloudSegmentation::getClusterTolerance()
{
    return clusterTolerance;
}


void DynCloudSegmentation::setMinClusterSize(int minClusterSize)
{
    this->minClusterSize = minClusterSize;
}


int DynCloudSegmentation::getMinClusterSize()
{
    return minClusterSize;
}


void DynCloudSegmentation::setMaxClusterSize(int maxClusterSize)
{
    this->maxClusterSize = maxClusterSize;
}


int DynCloudSegmentation::getMaxClusterSize()
{
    return maxClusterSize;
}


Publisher* DynCloudSegmentation::getPublisher()
{
    return pub;
}


DynCloudSegmentation::~DynCloudSegmentation()
{
    ;
}
