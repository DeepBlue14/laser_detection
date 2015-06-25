#include "MotionSeg.h"


bool MotionSeg::activateGuiBool = false;
int MotionSeg::sensitivityInt = 0;
int MotionSeg::blurInt = 0;


MotionSeg::MotionSeg()
{
    pub = new Publisher();
    centerPoint.x = -1;
    centerPoint.y = -1;
    centerPoint.z = -1;
}


void MotionSeg::callback(const sensor_msgs::ImageConstPtr& input)
{
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
    //- - - - - - - - - - - - - - - - - - - - -
    cv::cvtColor(cvImage, hsvImage, CV_BGR2HSV);
    /*cv::inRange(hsvImage,
                Scalar(0, 0, 0),
                Scalar(181, 361, 254),
                hsvImage);*/
    //- - - - - - - - - - - - - - - - - - - - -
    cv::imshow("Initial Image", cvImage);
    cv::waitKey(3);

    if(nextIterBool == true && getActivateGuiBool() == true)
    {
        filterByMotion(cvImage);
    }
    else
    {
        nextIterBool = true;
    }
    Mat t = cvImage;
    //- - - - - - - -
    prevImage = cvImage;

    cv_ptr->image = cvImage;

    //if(objectDetected)
        pub->publish(centerPoint);
}


void MotionSeg::filterByMotion(Mat nextImage)
{
    Mat grayImage1;
    Mat grayImage2;
    Mat differenceImage;
    Mat thresholdImage;

    cv::cvtColor(prevImage, grayImage1, COLOR_BGR2GRAY);
    cv::cvtColor(nextImage, grayImage2, COLOR_BGR2GRAY);
    cv::absdiff(grayImage1, grayImage2, differenceImage);
    cv::threshold(differenceImage, thresholdImage, getSensitivityInt(), 255, THRESH_BINARY);

    cv::imshow("Difference Image", differenceImage);
    cv::waitKey(3);
    cv::imshow("Threshold Image", thresholdImage);
    cv::waitKey(3);

    cv::blur(thresholdImage, thresholdImage, cv::Size(getBlurInt(), getBlurInt() ) );
    cv::threshold(thresholdImage, thresholdImage, getSensitivityInt(), 255, THRESH_BINARY);

    imshow("Final Threshold Image", thresholdImage);
    cv::waitKey(3);

    
    searchForMovement(thresholdImage, nextImage);
}


void MotionSeg::searchForMovement(Mat thresholdImage, Mat& cameraFeed)
{
    objectDetected = false;
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
    else
    {
        objectDetected = false;
    }
    
    if(objectDetected)
    {
        vector<vector<Point> > largestContourVec; // This is the moving object detected
        largestContourVec.push_back(contours.at(contours.size() - 1));
       
        //verifyColor(largestContourVec);

        // !!!approximate the center point of the object -- assuming the object at index 0 is correct!!!
        objectBoundingRectangle = boundingRect(largestContourVec.at(0));
        int xpos = objectBoundingRectangle.x + objectBoundingRectangle.width / 2;
        int ypos = objectBoundingRectangle.y + objectBoundingRectangle.height / 2;

        theObject[0] = xpos, theObject[1] = ypos;
        
        verifyColor(largestContourVec, Point(xpos, ypos));
    }

    int x = theObject[0];
    int y = theObject[1];
   
    Mat finalImage;
    cameraFeed.copyTo(finalImage);
   

    if(getActivateGuiBool() && objectDetected)
    {
	    circle(finalImage, Point(x,y), 20, Scalar(0,255,0),2);
	    line(finalImage, Point(x,y), Point(x,y-25), Scalar(0,255,0), 2);
	    line(finalImage, Point(x,y), Point(x,y+25), Scalar(0,255,0), 2);
	    line(finalImage, Point(x,y), Point(x-25,y), Scalar(0,255,0), 2);
	    line(finalImage, Point(x,y), Point(x+25,y), Scalar(0,255,0), 2);
    
        ostringstream convert;
        convert << "tracking object at (" << x << ", " << y << ")";
        setCenterPoint(x, y); // Set member variable
	    putText(finalImage, convert.str(), Point(x,y), 1, 1, Scalar(255,0,0), 2);
    }

    cv::imshow("Final Image", finalImage);
    cv::waitKey(3);
}


float MotionSeg::verifyColor(vector<vector<Point> > movingObjectCoors, Point centerPixel)
{
    float hSum = 0.0;
    float sSum = 0.0;
    float vSum = 0.0;
    float hAv = 0.0;
    float sAv = 0.0;
    float vAv = 0.0;
    float count = 0.0;
    float probability = 0.0;


    for(size_t i = 0; i < movingObjectCoors.at(0).size(); i++)
    {
        prevImage.at<cv::Vec3b>(movingObjectCoors.at(0).at(i).y, movingObjectCoors.at(0).at(i).x)[2] = 255;
    }
    

    for(size_t i = 0; i < 5; i++)
    {
        if(centerPixel.x > 5 && centerPixel.x < hsvImage.rows && centerPixel.y > 5 && centerPixel.y < hsvImage.cols)
        {
            
            hSum += hsvImage.at<cv::Vec3b>(centerPixel.y + i, centerPixel.x)[0];   // H
            sSum += hsvImage.at<cv::Vec3b>(centerPixel.y + i, centerPixel.x)[1];   // S
            vSum += hsvImage.at<cv::Vec3b>(centerPixel.y + i, centerPixel.x)[2];   // V
            
            hSum += hsvImage.at<cv::Vec3b>(centerPixel.y - i, centerPixel.x)[0];   // H
            sSum += hsvImage.at<cv::Vec3b>(centerPixel.y - i, centerPixel.x)[1];   // S
            vSum += hsvImage.at<cv::Vec3b>(centerPixel.y - i, centerPixel.x)[2];   // V
            
            hSum += hsvImage.at<cv::Vec3b>(centerPixel.y, centerPixel.x + i)[0];   // H
            sSum += hsvImage.at<cv::Vec3b>(centerPixel.y, centerPixel.x + i)[1];   // S
            vSum += hsvImage.at<cv::Vec3b>(centerPixel.y, centerPixel.x + i)[2];   // V
            
            hSum += hsvImage.at<cv::Vec3b>(centerPixel.y, centerPixel.x - i)[0];   // H
            sSum += hsvImage.at<cv::Vec3b>(centerPixel.y, centerPixel.x - i)[1];   // S
            vSum += hsvImage.at<cv::Vec3b>(centerPixel.y, centerPixel.x - i)[2];   // V
            
            count += 4;
            
            
            prevImage.at<cv::Vec3b>(centerPixel.y + i, centerPixel.x)[2] = 255;
            prevImage.at<cv::Vec3b>(centerPixel.y - i, centerPixel.x)[2] = 255;
            prevImage.at<cv::Vec3b>(centerPixel.y, centerPixel.x + i)[2] = 255;
            prevImage.at<cv::Vec3b>(centerPixel.y, centerPixel.x - i)[2] = 255;
        }
    }

    hAv = hSum / count;
    sAv = sSum / count;
    vAv = sSum / count;
    cout << "Average HSV: (" << hAv << ", " << sAv << ", " << vAv << ")" << endl;


    cv::imshow("Bound + 25px", prevImage);
    cv::waitKey(3);

    return probability;
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


void MotionSeg::setCenterPoint(int x, int y)
{
    centerPoint.x = x;
    centerPoint.y = y;
}


geometry_msgs::Point MotionSeg::getCenterPoint()
{
    return centerPoint;
}


Publisher* MotionSeg::getPublisher()
{
    return pub;
}


MotionSeg::~MotionSeg()
{
    ;
}
