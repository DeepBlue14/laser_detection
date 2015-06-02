#include "DynImageSegmentation.h"


bool DynImageSegmentation::activateGuiBool = false;
int DynImageSegmentation::redMinInt = 0;
int DynImageSegmentation::greenMinInt = 0;
int DynImageSegmentation::blueMinInt = 0;
int DynImageSegmentation::redMaxInt = 0;
int DynImageSegmentation::greenMaxInt = 0;
int DynImageSegmentation::blueMaxInt = 0;


DynImageSegmentation::DynImageSegmentation()
{
    pub = new Publisher();
}


void DynImageSegmentation::callback(const sensor_msgs::ImageConstPtr& input)
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
        cv_ptr = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::RGB8); // how about RGB16???
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what() );
    }
    
    cvImage = cv_ptr->image;
    //- - - - ----------------------------------------------------------------
    
    
    
    
    // creating various necessary staff
    cv::Mat matOriginal = cvImage;
    cv::Mat matOriginalHSV;
    cv::Mat matProcessed;

    std::vector<cv::Vec3f> vecCircles;
    std::vector<cv::Vec3f>::iterator itrCircles;

    char charEscKey = 0;

    cv::namedWindow("Original", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("Processed", CV_WINDOW_AUTOSIZE);

// creating trackbars
    cv::namedWindow("Trackbar", CV_WINDOW_AUTOSIZE);
    int hmin = 0, smin = 0, vmin = 0,
        hmax = 180, smax = 256, vmax = 256;
    cv::createTrackbar("H min:", "Trackbar", &hmin, hmax);
    cv::createTrackbar("H max:", "Trackbar", &hmax, hmax);
    cv::createTrackbar("S min:", "Trackbar", &smin, smax);
    cv::createTrackbar("S max:", "Trackbar", &smax, smax);
    cv::createTrackbar("V min:", "Trackbar", &vmin, vmax);
    cv::createTrackbar("V max:", "Trackbar", &vmax, vmax);
    
    // converting to HSV
        cv::cvtColor(matOriginal, matOriginalHSV, CV_BGR2HSV);

// spliting channels, changing any channel from "matOriginalHSVchannels[0/1/2]" to "matZero" to remove it
        cv::Mat matZero;
        cv::Mat matOriginalHS;
        matZero = cv::Mat::zeros(cv::Size(matOriginalHSV.cols, matOriginalHSV.rows), CV_8UC1);
        cv::vector<cv::Mat> matOriginalHSVchannels(3);
        cv::split(matOriginalHSV, matOriginalHSVchannels);
        cv::vector<cv::Mat> channels;
        channels.push_back(matOriginalHSVchannels[0]);
        channels.push_back(matOriginalHSVchannels[1]);
        channels.push_back(matZero);
        cv::merge(channels, matOriginalHS);

// search of laser spot
        cv::inRange(matOriginalHS, cv::Scalar(hmin, smin, vmin), cv::Scalar(hmax, smax, vmax), matProcessed);

// blur
        cv::GaussianBlur(matProcessed,
            matProcessed,
            cv::Size(5, 5),
            2.2);

// circles search 
        cv::HoughCircles(matProcessed,
            vecCircles,
            CV_HOUGH_GRADIENT,
            1,
            matProcessed.rows / 8,
            100,
            50,
            7,
            20);

// draw circles
        for (itrCircles = vecCircles.begin(); itrCircles != vecCircles.end(); itrCircles++) {
            std::cout << "position x = " << (*itrCircles)[0]
                << ", y = " << (*itrCircles)[1]
                << ", z = " << (*itrCircles)[2] << "\n";

            cv::circle(matOriginalHS,
                cv::Point((int)(*itrCircles)[0], (int)(*itrCircles)[1]),
                1,
                cv::Scalar(255, 0, 0),
                CV_FILLED);

            cv::circle(matOriginalHS,
                cv::Point((int)(*itrCircles)[0], (int)(*itrCircles)[1]),
                (int)(*itrCircles)[2],
                cv::Scalar(255, 255, 0),
                2);
    
    
            }
// show windows
        cv::imshow("Original", matOriginalHS);
        cv::imshow("Processed", matProcessed);
        charEscKey = cv::waitKey(10);
    
        cv::Mat matProcessedInv;
        cv::bitwise_not(matProcessed, matProcessedInv);
    
    //- - - - -----------------------------------------------------------------
    cv_ptr->image = cvImage;

    pub->publish(cv_ptr->toImageMsg() );
}


void DynImageSegmentation::setActivateGuiBool(bool activateGuiBool)
{
    this->activateGuiBool = activateGuiBool;
}


bool DynImageSegmentation::getActivateGuiBool()
{
    return activateGuiBool;
}


void DynImageSegmentation::setRedMinInt(int redMinInt)
{
    this->redMinInt = redMinInt;
}


int DynImageSegmentation::getRedMinInt()
{
    return redMinInt;
}


void DynImageSegmentation::setGreenMinInt(int greenMinInt)
{
    this->greenMinInt = greenMinInt;
}


int DynImageSegmentation::getGreenMinInt()
{
    return greenMinInt;
}


void DynImageSegmentation::setBlueMinInt(int blueMinInt)
{
    this->blueMinInt = blueMinInt;
}


int DynImageSegmentation::getBlueMinInt()
{
    return blueMinInt;
}


void DynImageSegmentation::setRedMaxInt(int redMaxInt)
{
    this->redMaxInt = redMaxInt;
}


int DynImageSegmentation::getRedMaxInt()
{
    return redMaxInt;
}


void DynImageSegmentation::setGreenMaxInt(int greenMaxInt)
{
    this->greenMaxInt = greenMaxInt;
}


int DynImageSegmentation::getGreenMaxInt()
{
    return greenMaxInt;
}


void DynImageSegmentation::setBlueMaxInt(int blueMaxInt)
{
    this->blueMaxInt = blueMaxInt;
}


int DynImageSegmentation::getBlueMaxInt()
{
    return blueMaxInt;
}


Publisher* DynImageSegmentation::getPublisher()
{
    return pub;
}


DynImageSegmentation::~DynImageSegmentation()
{
    ;
}
