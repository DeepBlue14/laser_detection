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
            
            /*
            cloud->points[pclCount].r = 0;
            cloud->points[pclCount].g = 0;
            cloud->points[pclCount].b = 0;
           */ 
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

            pclCount++;
        }
    }

    //for(size_t j = 0; j < cloud->points.size(); j++)
    //{
    //    if( !cloud->points[j].
   //}


    pub->publish(*cloud);
}


Publisher* GreenPart2::getPublisher()
{
    return pub;
}


GreenPart2::~GreenPart2()
{
    ;
}
