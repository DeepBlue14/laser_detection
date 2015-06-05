#include "MotionSeg.h"


bool MotionSeg::activateGuiBool = false;
int MotionSeg::sensitivityInt = 0;
int MotionSeg::blurInt = 0;


MotionSeg::MotionSeg()
{
    pub = new Publisher();
}


void MotionSeg::callback(const sensor_msgs::ImageConstPtr& input)
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
        cv_ptr = cv_bridge::toCvCopy(input/*, sensor_msgs::image_encodings::RGB16*//*RGB8*/); // TYPE_32SC4
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what() );
    }
    
    cvImage = cv_ptr->image;
    cv::imshow("Initial Image", cvImage);
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
    //- - - - - - -
    Mat motionMat = cvImage;
    if(nextIterBool == true && getActivateGuiBool() == true)
    {
        //cout << "Activating motion detection" << endl;
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


Mat MotionSeg::filterByMotion(Mat nextImage)
{
    //cv::imshow("Next Image", nextImage);
    //cv::waitKey(3);
    Mat grayImage1;
    Mat grayImage2;
    Mat differenceImage;
    Mat thresholdImage;
    //const static int SENSITIVITY_VALUE = 25;
    //const static int BLUR_SIZE = 10;

    cv::cvtColor(prevImage, grayImage1, COLOR_BGR2GRAY);
    cv::cvtColor(nextImage, grayImage2, COLOR_BGR2GRAY);
    cv::absdiff(grayImage1, grayImage2, differenceImage);
    cv::threshold(differenceImage, thresholdImage, getSensitivityInt()/*SENSITIVITY_VALUE*/, 255, THRESH_BINARY);

    cv::imshow("Difference Image", differenceImage);
    cv::waitKey(3);
    cv::imshow("Threshold Image", thresholdImage);
    cv::waitKey(3);

    cv::blur(thresholdImage, thresholdImage, cv::Size(getBlurInt()/*BLUR_SIZE*/, getBlurInt()/*BLUR_SIZE*/) );
    cv::threshold(thresholdImage, thresholdImage, getSensitivityInt()/*SENSITIVITY_VALUE*/, 255, THRESH_BINARY);

    //use this--the "final thresholdImage"
    imshow("Final Threshold Image", thresholdImage);
    cv::waitKey(3);

    
    searchForMovement(thresholdImage, nextImage);
    
    
    
    return thresholdImage;
}


void MotionSeg::searchForMovement(Mat thresholdImage, Mat& cameraFeed)
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
    {
        ROS_INFO("# of shapes found: %lu", contours.size() );
        objectDetected = true;
    }
    
    if(objectDetected)
    {
        //ROS_INFO("Found target");
        vector<vector<Point> > largestContourVec;
        largestContourVec.push_back(contours.at(contours.size() - 1)); //???

        objectBoundingRectangle = boundingRect(largestContourVec.at(0));
        int xpos = objectBoundingRectangle.x + objectBoundingRectangle.width / 2;
        int ypos = objectBoundingRectangle.y + objectBoundingRectangle.height / 2;

        theObject[0] = xpos, theObject[1] = ypos;
    }

    int x = theObject[0];
    int y = theObject[1];

    //cv::imshow("result", cameraFeed);
    //cv::waitKey(3);

    Mat finalImage;
    cameraFeed.copyTo(finalImage);
   
    //cv::imshow("Final Image", finalImage);
    //cv::waitKey(3);

    if(getActivateGuiBool() == true)
    {
	    circle(finalImage, Point(x,y), 20, Scalar(0,255,0),2);
	    line(finalImage, Point(x,y), Point(x,y-25), Scalar(0,255,0), 2);
	    line(finalImage, Point(x,y), Point(x,y+25), Scalar(0,255,0), 2);
	    line(finalImage, Point(x,y), Point(x-25,y), Scalar(0,255,0), 2);
	    line(finalImage, Point(x,y), Point(x+25,y), Scalar(0,255,0), 2);
    
	    //putText(cameraFeed,"Tracking object at (" + intToString(x)+","+intToString(y)+")",Point(x,y),1,1,Scalar(255,0,0),2);
        putText(finalImage, "tracking object", Point(x,y),1,1,Scalar(255,0,0),2);
    }
    cv::imshow("Final Image", finalImage);
    cv::waitKey(3);
}
        


void MotionSeg::setActivateGuiBool(bool activateGuiBool)
{
    this->activateGuiBool = activateGuiBool;
}


bool MotionSeg::getActivateGuiBool()
{
    return activateGuiBool;
}


void MotionSeg::setSensitivityInt(int sensitivityInt)
{
    this->sensitivityInt = sensitivityInt;
}


int MotionSeg::getSensitivityInt()
{
    return sensitivityInt;
}


void MotionSeg::setBlurInt(int blurInt)
{
    this->blurInt = blurInt;
}


int MotionSeg::getBlurInt()
{
    return blurInt;
}


Publisher* MotionSeg::getPublisher()
{
    return pub;
}


MotionSeg::~MotionSeg()
{
    ;
}
