#include "GreenDetectionCv.h"


bool GreenDetectionCv::activateGuiBool = false;
int GreenDetectionCv::redInt = 0;
int GreenDetectionCv::greenInt = 0;
int GreenDetectionCv::blueInt = 0;


GreenDetectionCv::GreenDetectionCv()
{
    pub = new Publisher();
}


void GreenDetectionCv::callback(const sensor_msgs::ImageConstPtr& input)
{
    //http://www.cse.sc.edu/~jokane/teaching/574/notes-images.pdf
    
    //convert to OpenCV type- - - - - - - - - - - - - - - - - -
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat cvImage;



    try
    {
        cv_ptr = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::RGB8); // how about RGB16???
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what() );
    }
    //pub->publish(input);
    
    

    pub->publish(cv_ptr->toImageMsg() );
}


void GreenDetectionCv::setActivateGuiBool(bool activateGuiBool)
{
    this->activateGuiBool = activateGuiBool;
}


bool GreenDetectionCv::getActivateGuiBool()
{
    return activateGuiBool;
}


void GreenDetectionCv::setRedInt(int redInt)
{
    this->redInt = redInt;
}


int GreenDetectionCv::getRedInt()
{
    return redInt;
}


void GreenDetectionCv::setGreenInt(int greenInt)
{
    this->greenInt = greenInt;
}


int GreenDetectionCv::getGreenInt()
{
    return greenInt;
}


void GreenDetectionCv::setBlueInt(int blueInt)
{
    this->blueInt = blueInt;
}


int GreenDetectionCv::getBlueInt()
{
    return blueInt;
}


Publisher* GreenDetectionCv::getPublisher()
{
    return pub;
}


GreenDetectionCv::~GreenDetectionCv()
{
    ;
}
