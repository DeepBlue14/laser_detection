#include "EvalPoint.h"


bool EvalPoint::activateGuiBool = false;
int EvalPoint::invalid_r = 255;
int EvalPoint::invalid_g = 0;
int EvalPoint::invalid_b = 0;
double EvalPoint::invalid_x = 1;
double EvalPoint::invalid_y = 1;
double EvalPoint::invalid_z = 1;
double EvalPoint::leafSize = 0.01;
int EvalPoint::maxIters = 100;
double EvalPoint::distThreshold = 0.02;
double EvalPoint::clusterTolerance = 0.02;
int EvalPoint::minClusterSize = 100;
int EvalPoint::maxClusterSize = 25000;


EvalPoint::EvalPoint()
{
    pub = new Publisher();
    imageIsReady = false;
    pointIsReady = false;
    cloudIsReady = false; 
}


void EvalPoint::imageCallback(const sensor_msgs::ImageConstPtr& image)
{
    this->image = image;
    imageIsReady = true; // ???Move this to point callback???
}


void EvalPoint::pointCallback(const geometry_msgs::PointConstPtr& pixelPoint)
{
    this->pixelPoint = pixelPoint;
    pointIsReady = true;
}


void EvalPoint::pointcloudCallback(PointCloud<PointXYZRGB>::Ptr cloud)
{
    this->cloud = cloud;
    cloudIsReady = true;

    //makesure that the image has been initialized; without this check
    //there would be a seg fault.
    if(imageIsReady == true && pointIsReady == true && cloudIsReady == true)
    {
        copyColorToCloud();
    }
}


void EvalPoint::copyColorToCloud()
{
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat cvImage;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(image/*, sensor_msgs::image_encodings::RGB8*/);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what() );
    }

    m_image = cvImage = cv_ptr->image;


    int pclCount = 0;
    for(size_t y = 0; y < cvImage.rows; y++)
    {
        for(size_t x = 0; x < cvImage.cols; x++)
        {
            Vec3b color = cvImage.at<Vec3b>(Point(x,y));
            cloud->points[pclCount].r = color.val[2];
            cloud->points[pclCount].g = color.val[1];
            cloud->points[pclCount].b = color.val[0];
            
            if(y == pixelPoint->y && x == pixelPoint->x) //???
            {
                realWorldCoorPoint.x = cloud->points[pclCount].x;
                realWorldCoorPoint.y = cloud->points[pclCount].y;
                realWorldCoorPoint.z = cloud->points[pclCount].z;
            }
            
            //cout << "(post) pixel z: " << cloud->points[pclCount].z << endl;
            pclCount++;
        } // end of outer for loop
    }

     cloud = euclideanClusterExtraction(cloud);
     cv_ptr->image = m_image;
     
    pub->publish(*cloud);
    //pub->publish(cv_ptr->toImageMsg() );
}


PointCloud<PointXYZRGB>::Ptr EvalPoint::euclideanClusterExtraction(PointCloud<PointXYZRGB>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (getLeafSize(), getLeafSize(), getLeafSize() );
    vg.filter (*cloud_filtered);
    //std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(getMaxIters() );
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
    ec.setClusterTolerance(getClusterTolerance() );
    ec.setMinClusterSize(getMinClusterSize() );
    ec.setMaxClusterSize(getMaxClusterSize() );
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

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
        float sumX = 0.0;
        float sumY = 0.0;
        float sumZ = 0.0;
        float maxX = -1.0;
        float minX = 100.0;
        float maxY = -1.0;
        float minY = 100.0;
        int count = 0;

        // Color all point clusters to red.
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            cloud_cluster->points[*pit] = cloud_filtered->points[*pit];
            
            cloud_cluster->points[*pit].r = 255;
            cloud_cluster->points[*pit].g = 255;
            cloud_cluster->points[*pit].b = 0;
            
            sumX += cloud_cluster->points[*pit].x;
            sumY += cloud_cluster->points[*pit].y;
            sumZ += cloud_cluster->points[*pit].z;

            if(cloud_cluster->points[*pit].x > maxX)
            {
                maxX = cloud_cluster->points[*pit].x;
            }

            if(cloud_cluster->points[*pit].x < minX)
            {
                minX = cloud_cluster->points[*pit].x;
            }
            
            if(cloud_cluster->points[*pit].y > maxY)
            {
                maxY = cloud_cluster->points[*pit].y;
            }

            if(cloud_cluster->points[*pit].y < minY)
            {
                minY = cloud_cluster->points[*pit].y;
            }
            


            count++;
        }
        
        const float ROBOT_MAX_RANGE = 1.0;
        
        geometry_msgs::Point clusterCenterPoint;
        clusterCenterPoint.x = sumX / count;
        clusterCenterPoint.y = sumY / count;
        clusterCenterPoint.z = sumZ / count;
        clusterCenterVec.push_back(clusterCenterPoint);
        //ROS_INFO("Object center: x=%f y=%f z=%f", clusterCenterPoint.x, clusterCenterPoint.y, clusterCenterPoint.z);
        
        ROS_INFO("average Z: %f", clusterCenterPoint.z);
        // Color clicked cluster to green
        if( (realWorldCoorPoint.x > minX) && (realWorldCoorPoint.x < maxX) && (realWorldCoorPoint.y > minY) && (realWorldCoorPoint.y < maxY) )
        {
            if(clusterCenterPoint.z < ROBOT_MAX_RANGE)
            {
                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                {
                    cloud_cluster->points[*pit].r = 0;
                    cloud_cluster->points[*pit].g = 255;
                    cloud_cluster->points[*pit].b = 0;
                }
                // !!! Change this, it will not resize relative to object size !!!
                rectangle(m_image, Point(pixelPoint->x - 35, pixelPoint->y - 35), Point(pixelPoint->x + 35, pixelPoint->y + 35), Scalar(0,200,0), 2);
            }
            else
            {
                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                {
                    cloud_cluster->points[*pit].r = 255;
                    cloud_cluster->points[*pit].g = 0;
                    cloud_cluster->points[*pit].b = 0;
                }
                rectangle(m_image, Point(pixelPoint->x - 35, pixelPoint->y - 35), Point(pixelPoint->x + 35, pixelPoint->y + 35), Scalar(200,0,0), 2);
            }
        }
        



    } // end of middle for loop
    
    //cout << "Number of clusters: " << cluster_indices.size() << endl;

    return cloud_cluster;
}


void EvalPoint::conditionalEuclideanClustering(PointCloud<PointXYZRGB>::Ptr cloud)
{
    ;
}


void EvalPoint::setActivateGuiBool(bool activateGuiBool)
{
    this->activateGuiBool = activateGuiBool;
}


bool EvalPoint::getActivateGuiBool()
{
    return activateGuiBool;
}


void EvalPoint::setInvalid_r(int invalid_r)
{
    this->invalid_r = invalid_r;
}


int EvalPoint::getInvalid_r()
{
    return invalid_r;
}


void EvalPoint::setInvalid_g(int invalid_g)
{
    this->invalid_g = invalid_g;
}


int EvalPoint::getInvalid_g()
{
    return invalid_g;
}


void EvalPoint::setInvalid_b(int invalid_b)
{
    this->invalid_b = invalid_b;
}


int EvalPoint::getInvalid_b()
{
    return invalid_b;
}


void EvalPoint::setInvalid_x(double invalid_x)
{
    this->invalid_x = invalid_x;
}


double EvalPoint::getInvalid_x()
{
    return invalid_x;
}


void EvalPoint::setInvalid_y(double invalid_y)
{
    this->invalid_y = invalid_y;
}


double EvalPoint::getInvalid_y()
{
    return invalid_y;
}


void EvalPoint::setInvalid_z(double invalid_z)
{
    this->invalid_z = invalid_z;
}


double EvalPoint::getInvalid_z()
{
    return invalid_z;
}


void EvalPoint::setLeafSize(double leafSize)
{
    this->leafSize = leafSize;
}


double EvalPoint::getLeafSize()
{
    return leafSize;
}


void EvalPoint::setMaxIters(int maxIters)
{
    this->maxIters = maxIters;
}


int EvalPoint::getMaxIters()
{
    return maxIters;
}


void EvalPoint::setDistThreshold(double distThreshold)
{
    this->distThreshold = distThreshold;
}


double EvalPoint::getDistThreshold()
{
    return distThreshold;
}


void EvalPoint::setClusterTolerance(double clusterTolerance)
{
    this->clusterTolerance = clusterTolerance;
}


double EvalPoint::getClusterTolerance()
{
    return clusterTolerance;
}


void EvalPoint::setMinClusterSize(int minClusterSize)
{
    this->minClusterSize = minClusterSize;
}


int EvalPoint::getMinClusterSize()
{
    return minClusterSize;
}


void EvalPoint::setMaxClusterSize(int maxClusterSize)
{
    this->maxClusterSize = maxClusterSize;
}


int EvalPoint::getMaxClusterSize()
{
    return maxClusterSize;
}


Publisher* EvalPoint::getPublisher()
{
    return pub;
}


EvalPoint::~EvalPoint()
{
    ;
}
