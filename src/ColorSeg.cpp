#include "ColorSeg.h"


bool ColorSeg::activateGuiBool = false;
int ColorSeg::redMinInt = 0;
int ColorSeg::greenMinInt = 0;
int ColorSeg::blueMinInt = 0;
int ColorSeg::redMaxInt = 0;
int ColorSeg::greenMaxInt = 0;
int ColorSeg::blueMaxInt = 0;


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
    cv::imshow("Initial Image", cvImage);
    cv::waitKey(3);


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

    cv::imshow("Final Image", cvImage);
    cv::waitKey(3);

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


Publisher* ColorSeg::getPublisher()
{
    return pub;
}


ColorSeg::~ColorSeg()
{
    ;
}
