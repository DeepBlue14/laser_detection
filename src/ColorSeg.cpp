#include "ColorSeg.h"


//Force initialization of static vars
bool ColorSeg::activateGuiBool = false;
int ColorSeg::redMinInt = 0;
int ColorSeg::greenMinInt = 0;
int ColorSeg::blueMinInt = 0;
int ColorSeg::redMaxInt = 0;
int ColorSeg::greenMaxInt = 0;
int ColorSeg::blueMaxInt = 0;
int ColorSeg::hMinInt = 0;
int ColorSeg::sMinInt = 0;
int ColorSeg::vMinInt = 0;
int ColorSeg::hMaxInt = 0;
int ColorSeg::sMaxInt = 0;
int ColorSeg::vMaxInt = 0;


ColorSeg::ColorSeg()
{
    pub = new Publisher();
}


void ColorSeg::callback(const sensor_msgs::ImageConstPtr& input)
{
    //initialize variables
    minX = 1000000;
    minY = 1000000;
    maxX = -100000;
    maxY = -100000;

    for(size_t i = 0; i < validXVec.size(); i++)
    {
        validXVec.pop_back();
        validYVec.pop_back();
    }

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
    
    cv::imshow("Initial RGB Image", cvImage);
    cv::waitKey(3);
    
    cv::Mat hsvImage;
    cvtColor(cvImage, hsvImage, COLOR_RGB2HSV);
    cv::imshow("Initial HSV Image", hsvImage);
    cv::waitKey(3);
    
    cv::inRange(cvImage,
                Scalar(getRedMinInt(), getGreenMinInt(), getBlueMinInt() ),
                Scalar(getRedMaxInt(), getGreenMaxInt(), getBlueMaxInt() ),
                cvImage);
    cv::imshow("Final RGB Image", cvImage);
    cv::waitKey(3);
    
    cv::inRange(hsvImage,
                Scalar(getHMinInt(), getSMinInt(), getVMinInt() ),
                Scalar(getHMaxInt(), getSMaxInt(), getVMaxInt() ),
                hsvImage);
    cv::imshow("Final HSV Image", hsvImage);
    cv::waitKey(3);

    

    
    
    
    
/*
 // Circle detection
 //http://www.pyimagesearch.com/2014/07/21/detecting-circles-images-using-opencv-hough-circles/
    //- - - - - - -
    Mat image_gray;
    cvtColor(cvImage, image_gray, CV_BGR2GRAY);

    /// Reduce the noise
    GaussianBlur(image_gray, image_gray, Size(9, 9), 2, 2);

    vector<Vec3f> circles;

    /// Apply the Hough Transform to find the circle(s)
    HoughCircles(image_gray, circles, CV_HOUGH_GRADIENT, 5, image_gray.rows/6, 200, 10, 1, 10);
    ROS_INFO("Number of circles detected:%lu", circles.size() );
    for(size_t i = 0; i < circles.size(); i++)
    {
        //ROS_INFO("looping...");
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        ROS_INFO("\tAdding circle of radius: %d", radius);
        // circle center
        circle(cvImage, center, 3, Scalar(255, 0, 0), -1);
        // circle outline
        circle(cvImage, center, radius, Scalar(0, 0, 255), 1);
    }
*/

    

    cv_ptr->image = cvImage;
    pub->publish(cv_ptr->toImageMsg() );
}


void ColorSeg::setActivateGuiBool(bool activateGuiBool)
{
    this->activateGuiBool = activateGuiBool;
}


bool ColorSeg::getActivateGuiBool()
{
    return activateGuiBool;
}


// RGB
void ColorSeg::setRedMinInt(int redMinInt)
{
    this->redMinInt = redMinInt;
}


int ColorSeg::getRedMinInt()
{
    return redMinInt;
}


void ColorSeg::setGreenMinInt(int greenMinInt)
{
    this->greenMinInt = greenMinInt;
}


int ColorSeg::getGreenMinInt()
{
    return greenMinInt;
}


void ColorSeg::setBlueMinInt(int blueMinInt)
{
    this->blueMinInt = blueMinInt;
}


int ColorSeg::getBlueMinInt()
{
    return blueMinInt;
}


void ColorSeg::setRedMaxInt(int redMaxInt)
{
    this->redMaxInt = redMaxInt;
}


int ColorSeg::getRedMaxInt()
{
    return redMaxInt;
}


void ColorSeg::setGreenMaxInt(int greenMaxInt)
{
    this->greenMaxInt = greenMaxInt;
}


int ColorSeg::getGreenMaxInt()
{
    return greenMaxInt;
}


void ColorSeg::setBlueMaxInt(int blueMaxInt)
{
    this->blueMaxInt = blueMaxInt;
}


int ColorSeg::getBlueMaxInt()
{
    return blueMaxInt;
}


// HSV
void ColorSeg::setHMinInt(int hMinInt)
{
    this->hMinInt = hMinInt;
}


int ColorSeg::getHMinInt()
{
    return hMinInt;
}


void ColorSeg::setSMinInt(int sMinInt)
{
    this->sMinInt = sMinInt;
}


int ColorSeg::getSMinInt()
{
    return sMinInt;
}


void ColorSeg::setVMinInt(int vMinInt)
{
    this->vMinInt = vMinInt;
}


int ColorSeg::getVMinInt()
{
    return vMinInt;
}


void ColorSeg::setHMaxInt(int hMaxInt)
{
    this->hMaxInt = hMaxInt;
}


int ColorSeg::getHMaxInt()
{
    return hMaxInt;
}


void ColorSeg::setSMaxInt(int sMaxInt)
{
    this->sMaxInt = sMaxInt;
}


int ColorSeg::getSMaxInt()
{
    return sMaxInt;
}


void ColorSeg::setVMaxInt(int vMaxInt)
{
    this->vMaxInt = vMaxInt;
}


int ColorSeg::getVMaxInt()
{
    return vMaxInt;
}


Publisher* ColorSeg::getPublisher()
{
    return pub;
}


ColorSeg::~ColorSeg()
{
    ;
}
