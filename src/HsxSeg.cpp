#include "HsxSeg.h"


bool HsxSeg::activateGuiBool = false;
int HsxSeg::hMinInt = 0.0;
int HsxSeg::sMinInt = 0.0;
int HsxSeg::vMinInt = 0.0;
int HsxSeg::hMaxInt = 0.0;
int HsxSeg::sMaxInt = 0.0;
int HsxSeg::vMaxInt = 0.0;
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
    //swap R <-> B
    int tmpColor;
    for(size_t a = 0; a < cvImage.rows; a++)
    {
        for(size_t b = 0; b < cvImage.cols; b++)
        {
            tmpColor = cvImage.at<cv::Vec3b>(a, b)[0];
            cvImage.at<cv::Vec3b>(a, b)[0] = cvImage.at<cv::Vec3b>(a, b)[2];
            cvImage.at<cv::Vec3b>(a, b)[2] = tmpColor;
        }
    }
    cv::imshow("Initial Image", cvImage);
    cv::waitKey(3);

    //- - - - - - - - - - - - - -
    //cv::blur(thresholdImage, thresholdImage, cv::Size(getBlurInt(), getBlurInt() ) );
    //cv::threshold(thresholdImage, thresholdImage, getSensitivityInt(), 255, THRESH_BINARY);
    
    cv::Mat hsxImage;
    cv::cvtColor(cvImage, hsxImage, CV_BGR2HSV);
    //cv::cvtColor(cvImage, hsxImage, CV_BGR2HLS);
   
    cv::Mat inrangeImage;
    
    //cv::blur(hsxImage, hsxImage, cv::Size(1, 1) );
    //cv::threshold(hsxImage, hsxImage, getSensitivityInt(), 255, THRESH_BINARY);
    
    cv::inRange(hsxImage,
                Scalar(getHMinInt(), getSMinInt(), getVMinInt() ),
                Scalar(getHMaxInt(), getSMaxInt(), getVMaxInt() ),
                inrangeImage);
    
    //cv::blur(inrangeImage, inrangeImage, cv::Size(1, 1) );
    //cv::threshold(inrangeImage, inrangeImage, getSensitivityInt(), 255, THRESH_BINARY);

    //- - - - - - - - - - - - - -
    
    cv::imshow("HSV Image", hsxImage);
    cv::waitKey(3);
    cv::imshow("inRange Image", inrangeImage);
    cv::waitKey(3); 
    
    //^^^^^^^^^^^^^^^^^
    bool objectDetected = false;
    Mat temp;
    inrangeImage.copyTo(temp);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    int theObject[2] = {0,0};
    Rect objectBoundingRectangle = Rect(0,0,0,0);

    findContours(temp,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );

    if(contours.size() > 0)
    {
        ROS_INFO("# of shapes found: %lu", contours.size() );
        objectDetected = true;
    }
    else
    {
        objectDetected = false;
    }
    
    if(contours.size() == 1)
    {
        ROS_INFO("# of points in contour: %lu", contours.at(0).size() );
    }
    //^^^^^^^^^^^^^^^^^

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


void HsxSeg::setHMinInt(int hMinInt)
{
    this->hMinInt = hMinInt;
}


int HsxSeg::getHMinInt()
{
    return hMinInt;
}


void HsxSeg::setSMinInt(int sMinInt)
{
    this->sMinInt = sMinInt;
}


int HsxSeg::getSMinInt()
{
    return sMinInt;
}


void HsxSeg::setVMinInt(int vMinInt)
{
    this->vMinInt = vMinInt;
}


int HsxSeg::getVMinInt()
{
    return vMinInt;
}


void HsxSeg::setHMaxInt(int hMaxInt)
{
    this->hMaxInt = hMaxInt;
}


int HsxSeg::getHMaxInt()
{
    return hMaxInt;
}


void HsxSeg::setSMaxInt(int sMaxInt)
{
    this->sMaxInt = sMaxInt;
}


int HsxSeg::getSMaxInt()
{
    return sMaxInt;
}


void HsxSeg::setVMaxInt(int vMaxInt)
{
    this->vMaxInt = vMaxInt;
}


int HsxSeg::getVMaxInt()
{
    return vMaxInt;
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
