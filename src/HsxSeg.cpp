#include "HsxSeg.h"


HsxSeg::HsxSeg()
{
    pub = new Publisher();
}


void HsxSeg::callback(const sensor_msgs::ImageConstPtr& input)
{

    //http://www.cse.sc.edu/~jokane/teaching/574/notes-images.pdf
    
    //convert to OpenCV type- - - - - - - - - - - - - - - - - -
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat cvImage;



    try
    {
        //http://docs.ros.org/indigo/api/sensor_msgs/html/image__encodings_8h_source.html
        cv_ptr = cv_bridge::toCvCopy(input/*, sensor_msgs::image_encodings::RGB16*/); // TYPE_32SC4
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what() );
    }
    
    cvImage = cv_ptr->image;
    cv::imshow("Initial Image", cvImage);
    cv::waitKey(3);

    //- - - - - - - - - - - - - -
    cv::Mat hsxImage;
    cv::cvtColor(cvImage, hsxImage, CV_BGR2HSV);
    //cv::cvtColor(cvImage, hsxImage, CV_BGR2HSL);
    //- - - - - - - - - - - - - -
    
    cv::imshow("Final Image", hsxImage);
    cv::waitKey(3);

    cv_ptr->image = cvImage;

    pub->publish(cv_ptr->toImageMsg() );
}



Publisher* HsxSeg::getPublisher()
{
    return pub;
}


HsxSeg::~HsxSeg()
{
    ;
}
