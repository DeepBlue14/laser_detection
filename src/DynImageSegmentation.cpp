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
    //- - - - - - -
    Mat motionMat = cvImage;
    if(nextIterBool == true && getActivateGuiBool() == true)
    {
        motionMat = filterByMotion(cvImage);
    }
    else
    {
        motionMat = cvImage;
        
        nextIterBool = true;
    }
    Mat t = cvImage;
    //- - - - - - - -
    prevImage = cvImage;

    cv_ptr->image = motionMat;

    pub->publish(cv_ptr->toImageMsg() );
}


Mat DynImageSegmentation::filterByMotion(Mat nextImage)
{
    //cv::imshow("Next Image", nextImage);
    //cv::waitKey(3);
    Mat grayImage1;
    Mat grayImage2;
    Mat differenceImage;
    Mat thresholdImage;
    const static int SENSITIVITY_VALUE = 20;
    const static int BLUR_SIZE = 10;

    cv::cvtColor(prevImage, grayImage1, COLOR_BGR2GRAY);
    cv::cvtColor(nextImage, grayImage2, COLOR_BGR2GRAY);
    cv::imshow("Next Image", nextImage);
    cv::waitKey(3);
    cv::absdiff(grayImage1, grayImage2, differenceImage);
    cv::threshold(differenceImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);

    cv::imshow("Difference Image", differenceImage);
    cv::waitKey(3);
    cv::imshow("Threshold Image", thresholdImage);
    cv::waitKey(3);

    cv::blur(thresholdImage, thresholdImage, cv::Size(BLUR_SIZE, BLUR_SIZE) );
    cv::threshold(thresholdImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);

    imshow("Final Threshold Image", thresholdImage);
    cv::waitKey(3);

    
    searchForMovement(thresholdImage, nextImage);
    
    
    
    return thresholdImage;
}


void DynImageSegmentation::searchForMovement(Mat thresholdImage, Mat& cameraFeed)
{
    bool objectDetected = false;
    Mat temp;
    thresholdImage.copyTo(temp);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    int theObject[2] = {0,0};
    Rect objectBoundingRectangle = Rect(0,0,0,0);

    findContours(temp,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );

    if(contours.size() > 0)
        objectDetected = true;

    if(objectDetected)
    {
        //ROS_INFO("Found target");
        vector<vector<Point> > largestContourVec;
        largestContourVec.push_back(contours.at(contours.size() - 1));

        objectBoundingRectangle = boundingRect(largestContourVec.at(0));
        int xpos = objectBoundingRectangle.x + objectBoundingRectangle.width / 2;
        int ypos = objectBoundingRectangle.y + objectBoundingRectangle.height / 2;

        theObject[0] = xpos, theObject[1] = ypos;
    }

    int x = theObject[0];
    int y = theObject[1];

    cv::imshow("result", cameraFeed);
    cv::waitKey(3);

	circle(cameraFeed,Point(x,y),20,Scalar(0,255,0),2);
	/*line(cameraFeed,Point(x,y),Point(x,y-25),Scalar(0,255,0),2);
	line(cameraFeed,Point(x,y),Point(x,y+25),Scalar(0,255,0),2);
	line(cameraFeed,Point(x,y),Point(x-25,y),Scalar(0,255,0),2);
	line(cameraFeed,Point(x,y),Point(x+25,y),Scalar(0,255,0),2);
    
	//putText(cameraFeed,"Tracking object at (" + intToString(x)+","+intToString(y)+")",Point(x,y),1,1,Scalar(255,0,0),2);
    putText(cameraFeed, "tracking object", Point(x,y),1,1,Scalar(255,0,0),2);
    */
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
