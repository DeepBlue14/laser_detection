#include "ShapeSeg.h"


bool ShapeSeg::activateGuiBool = false;
int ShapeSeg::sensitivityInt = 0;
int ShapeSeg::blurInt = 0;


ShapeSeg::ShapeSeg()
{
    pub = new Publisher();
}


void ShapeSeg::callback(const sensor_msgs::ImageConstPtr& input)
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


Mat ShapeSeg::filterByMotion(Mat nextImage)
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

    
    //searchForMovement(thresholdImage, nextImage);
    
    //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    vector<Vec3f> circles;
    
    // Apply the Hough Transform to find the circles
    HoughCircles(thresholdImage, circles, CV_HOUGH_GRADIENT, 1, thresholdImage.rows/8, 200, 100, 0, 0);
    
    for(size_t i = 0; i < circles.size(); i++)
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        
        // Circle center
        circle(nextImage, center, 3, Scalar(0, 255, 0), -1, 8, 0);
        
        // Circles outline
        circle(nextImage, center, radius, Scalar(0, 0, 255), 3, 8, 0);
    }
    
    cv::imshow("Final Shape Image", nextImage);
    cv::waitKey(3);
    
    //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    
    return thresholdImage;
}


/*
void ShapeSeg::searchForMovement(Mat thresholdImage, Mat& cameraFeed)
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

	circle(finalImage, Point(x,y), 20, Scalar(0,255,0),2);
	line(finalImage, Point(x,y), Point(x,y-25), Scalar(0,255,0), 2);
	line(finalImage, Point(x,y), Point(x,y+25), Scalar(0,255,0), 2);
	line(finalImage, Point(x,y), Point(x-25,y), Scalar(0,255,0), 2);
	line(finalImage, Point(x,y), Point(x+25,y), Scalar(0,255,0), 2);
    
	//putText(cameraFeed,"Tracking object at (" + intToString(x)+","+intToString(y)+")",Point(x,y),1,1,Scalar(255,0,0),2);
    putText(finalImage, "tracking object", Point(x,y),1,1,Scalar(255,0,0),2);
    
    cv::imshow("Final Image", finalImage);
    cv::waitKey(3);
}
*/       


void ShapeSeg::setActivateGuiBool(bool activateGuiBool)
{
    this->activateGuiBool = activateGuiBool;
}


bool ShapeSeg::getActivateGuiBool()
{
    return activateGuiBool;
}


void ShapeSeg::setSensitivityInt(int sensitivityInt)
{
    this->sensitivityInt = sensitivityInt;
}


int ShapeSeg::getSensitivityInt()
{
    return sensitivityInt;
}


void ShapeSeg::setBlurInt(int blurInt)
{
    this->blurInt = blurInt;
}


int ShapeSeg::getBlurInt()
{
    return blurInt;
}


Publisher* ShapeSeg::getPublisher()
{
    return pub;
}


ShapeSeg::~ShapeSeg()
{
    ;
}
