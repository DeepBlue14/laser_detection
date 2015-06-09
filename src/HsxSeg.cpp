#include "HsxSeg.h"


bool HsxSeg::activateGuiBool = false;
double HsxSeg::hMinDbl = 0.0;
double HsxSeg::sMinDbl = 0.0;
double HsxSeg::vMinDbl = 0.0;
double HsxSeg::hMaxDbl = 0.0;
double HsxSeg::sMaxDbl = 0.0;
double HsxSeg::vMaxDbl = 0.0;
int HsxSeg::sensitivityInt = 0;
int HsxSeg::blurInt = 0;


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
   
    cv::Mat inrangeImage; 
    cv::inRange(hsxImage,
                Scalar(getHMinDbl(), getSMinDbl(), getVMinDbl() ),
                Scalar(getHMaxDbl(), getSMaxDbl(), getVMaxDbl() ),
                inrangeImage);
    


    //- - - - - - - - - - - - - -
    
    cv::imshow("HSV Image", hsxImage);
    cv::waitKey(3);
    cv::imshow("inRange Image", inrangeImage);
    cv::waitKey(3); 

    cv_ptr->image = cvImage;

    pub->publish(cv_ptr->toImageMsg() );
}


void HsxSeg::setActivateGuiBool(bool activateGuiBool)
{
    this->activateGuiBool = activateGuiBool;
}


bool HsxSeg::getActivateGuiBool()
{
    return activateGuiBool;
}


void HsxSeg::setHMinDbl(double hMinDbl)
{
    this->hMinDbl = hMinDbl;
}


double HsxSeg::getHMinDbl()
{
    return hMinDbl;
}


void HsxSeg::setSMinDbl(double sMinDbl)
{
    this->sMinDbl = sMinDbl;
}


double HsxSeg::getSMinDbl()
{
    return sMinDbl;
}


void HsxSeg::setVMinDbl(double vMinDbl)
{
    this->vMinDbl = vMinDbl;
}


double HsxSeg::getVMinDbl()
{
    return vMinDbl;
}


void HsxSeg::setHMaxDbl(double hMaxDbl)
{
    this->hMaxDbl = hMaxDbl;
}


double HsxSeg::getHMaxDbl()
{
    return hMaxDbl;
}


void HsxSeg::setSMaxDbl(double sMaxDbl)
{
    this->sMaxDbl = sMaxDbl;
}


double HsxSeg::getSMaxDbl()
{
    return sMaxDbl;
}


void HsxSeg::setVMaxDbl(double vMaxDbl)
{
    this->vMaxDbl = vMaxDbl;
}


double HsxSeg::getVMaxDbl()
{
    return vMaxDbl;
}


void HsxSeg::setSensitivityInt(int sensitivityInt)
{
    this->sensitivityInt = sensitivityInt;
}


int HsxSeg::getSensitivityInt()
{
    return sensitivityInt;
}


void HsxSeg::setBlurInt(int blurInt)
{
    this->blurInt;
}


int HsxSeg::getBlurInt()
{
    return blurInt;
}


Publisher* HsxSeg::getPublisher()
{
    return pub;
}


HsxSeg::~HsxSeg()
{
    ;
}
