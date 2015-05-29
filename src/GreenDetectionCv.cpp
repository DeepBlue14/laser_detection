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

    for(size_t y = 0; y < cvImage.rows; y++)
    {
        for(size_t x = 0; x < cvImage.cols; x++)
        {
            Vec3b color = cvImage.at<Vec3b>(Point(x,y));

            float red = color.val[2];
            float green = color.val[1];
            float blue = color.val[0];
            //cout << red << "\n" << green << "\n" << blue << "\n\n\n" << endl;
            //- - -
            
            if( (red < redMinInt) || (red > redMaxInt) )
            {
                red = 0;
                green = 0;
                blue = 0;
            }
            else if( (green < greenMinInt) || (green > greenMaxInt) )
            {
                red = 0;
                green = 0;
                blue = 0;
            }
            else if( (blue < blueMinInt) || (blue > blueMaxInt) )
            {
                red = 0;
                green = 0;
                blue = 0;
            }
            else
            {
                validXVec.push_back(x);
                validYVec.push_back(y);
                if(x < minX)
                {
                    minX = x;
                }
                if(x > maxX)
                {
                    maxX = x;
                }
                if(y < minY)
                {
                    minY = y;
                }
                if(y > maxY)
                {
                    maxY = y;
                }
            }

            color.val[2] = red;
            color.val[1] = green;
            color.val[0] = blue;
            cvImage.at<Vec3b>(Point(x,y)) = color;
        }
    }
/*
 * http://www.pyimagesearch.com/2014/07/21/detecting-circles-images-using-opencv-hough-circles/
    Mat image_gray;
    cvtColor(cvImage, image_gray, CV_BGR2GRAY);
    vector<Vec3f> circles;
    HoughCircles(image_gray, circles, CV_HOUGH_GRADIENT, 5, image_gray.rows/6, 200, 20, 10, 40);
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

    //- - - - - - -
*/

    cv_ptr->image = cvImage;

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
