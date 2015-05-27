#include "GreenDetectionCv.h"


bool GreenDetectionCv::activateGuiBool = false;
int GreenDetectionCv::redMinInt = 0;
int GreenDetectionCv::greenMinInt = 0;
int GreenDetectionCv::blueMinInt = 0;
int GreenDetectionCv::redMaxInt = 0;
int GreenDetectionCv::greenMaxInt = 0;
int GreenDetectionCv::blueMaxInt = 0;


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
    
   cvImage = cv_ptr->image;
   //- - - - - - - -
    
   uchar r, g, b; //int instead of uchar???
   for(size_t i = 0; i < cvImage.rows; i++)
   {
       cv::Vec3d* pixel = cvImage.ptr<cv::Vec3d>(i); // point to first pixel in row
     for(size_t  k = 0; k < cvImage.cols; k++)
     {
        //r = pixel[k][2];
        //g = pixel[k][1];
        //r = pixel[k][0];

        if(pixel[k][2] < greenMinInt)
        {
            pixel[k][2] = 0;
            pixel[k][1] = 0;
            pixel[k][0] = 0;
        }
        
        

        
     }
   }
    
    
   //- - - - - - - -

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


void GreenDetectionCv::setRedMinInt(int redMinInt)
{
    this->redMinInt = redMinInt;
}


int GreenDetectionCv::getRedMinInt()
{
    return redMinInt;
}


void GreenDetectionCv::setGreenMinInt(int greenMinInt)
{
    this->greenMinInt = greenMinInt;
}


int GreenDetectionCv::getGreenMinInt()
{
    return greenMinInt;
}


void GreenDetectionCv::setBlueMinInt(int blueMinInt)
{
    this->blueMinInt = blueMinInt;
}


int GreenDetectionCv::getBlueMinInt()
{
    return blueMinInt;
}


void GreenDetectionCv::setRedMaxInt(int redMaxInt)
{
    this->redMaxInt = redMaxInt;
}


int GreenDetectionCv::getRedMaxInt()
{
    return redMaxInt;
}


void GreenDetectionCv::setGreenMaxInt(int greenMaxInt)
{
    this->greenMaxInt = greenMaxInt;
}


int GreenDetectionCv::getGreenMaxInt()
{
    return greenMaxInt;
}


void GreenDetectionCv::setBlueMaxInt(int blueMaxInt)
{
    this->blueMaxInt = blueMaxInt;
}


int GreenDetectionCv::getBlueMaxInt()
{
    return blueMaxInt;
}


Publisher* GreenDetectionCv::getPublisher()
{
    return pub;
}


GreenDetectionCv::~GreenDetectionCv()
{
    ;
}
