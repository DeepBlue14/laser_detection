#include "MotionSeg.h"


bool MotionSeg::activateGuiBool = false; //old: false
int MotionSeg::sensitivityInt = 20;
int MotionSeg::blurInt = 25;


MotionSeg::MotionSeg()
{
    pub = new Publisher();
    centerPoint.x = -1;
    centerPoint.y = -1;
    centerPoint.z = -1;
}


/**
 * convert the ros image to an OpenCV matrix and ship it off for evaluation.
 *
 */
void MotionSeg::callback(const sensor_msgs::ImageConstPtr& input)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat cvImage;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(input);
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
    
    cv::cvtColor(cvImage, hsvImage, CV_BGR2HSV);
    //cv::imshow("Initial Image", cvImage);
    //cv::waitKey(3);

    if(nextIterBool == true && getActivateGuiBool() == true)
    {
        filter(cvImage);
    }
    else
    {
        nextIterBool = true;
    }
    
    //Mat t = cvImage;
    //prevImage = cvImage; //***here there be dragons*** --opencv implicitly uses reff, not value
    cvImage.copyTo(prevImage);
    cv_ptr->image = cvImage;
    pub->publish(centerPoint);
    activateGuiBool = true;
}


/**
 * Filter the image and pass it on.
 */
void MotionSeg::filter(Mat nextImage)
{
    Mat grayImage1;
    Mat grayImage2;
    Mat differenceImage;
    Mat thresholdImage;
    cv::cvtColor(prevImage, grayImage1, COLOR_BGR2GRAY);
    cv::cvtColor(nextImage, grayImage2, COLOR_BGR2GRAY);
    cv::absdiff(grayImage1, grayImage2, differenceImage);
    cv::threshold(differenceImage, thresholdImage, getSensitivityInt(), 255, THRESH_BINARY);
    cv::blur(thresholdImage, thresholdImage, cv::Size(getBlurInt(), getBlurInt() ) ); //FIXME: this breaks on start
    cv::threshold(thresholdImage, thresholdImage, getSensitivityInt(), 255, THRESH_BINARY);

    //imshow("Final Threshold Image", thresholdImage);
    //cv::waitKey(3);

    searchForMovement(thresholdImage, nextImage);
}


/**
 * Evaluate the moving shapes.
 * Baseline: They should be (, ) with a 
 */
void MotionSeg::searchForMovement(Mat thresholdImage, Mat& cameraFeed)
{
    objectDetected = false;
    Mat temp;
    thresholdImage.copyTo(temp);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    int theObject[2] = {0, 0};

    findContours(temp, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    if(contours.size() > 0)
    {
        ROS_INFO("# of shapes found: %lu", contours.size() );
        objectDetected = true;
    }
    else
    {
        objectDetected = false; //this is redundent; objectDetected is already set to false by default
    }
    
    if(objectDetected)
    {
        vector<vector<Point> > largestContourVec;
        if(contours.size() > 0)
        {   
            largestContourVec.push_back(contours.at(contours.size() - 1) ); //FIXME: this assumes that the last moving object in the vector is the correct one--an assumption that may not always be true
        }
        else
        {   cerr << "Haha! I have found the segfault!" << endl;
            exit(1);
        }
            

        // !!!approximate the center point of the object -- assuming the object at index 0 is correct!!!
        Rect objectBoundingRectangle = Rect(0, 0, 0, 0);
        objectBoundingRectangle = boundingRect(largestContourVec.at(0) );
        int xpos = objectBoundingRectangle.x + objectBoundingRectangle.width / 2;
        int ypos = objectBoundingRectangle.y + objectBoundingRectangle.height / 2;
        
        theObject[0] = xpos, theObject[1] = ypos;
        verifyColor(largestContourVec, Point(xpos, ypos) );
    }
    
    int x = theObject[0];
    int y = theObject[1];
   
    Mat finalImage;
    cameraFeed.copyTo(finalImage);

    //if(getActivateGuiBool() && objectDetected && closeEnough(x, y, getCenterPoint()) ) // ***James was here***
    if(objectDetected)
    {
        cout << "objectDetected = true" << endl;
	    circle(finalImage, Point(x, y), 20, Scalar(0, 255, 0), 2);
	    line(finalImage, Point(x, y), Point(x, y-25), Scalar(0, 255, 0), 2);
	    line(finalImage, Point(x, y), Point(x, y+25), Scalar(0, 255, 0), 2);
	    line(finalImage, Point(x, y), Point(x-25, y), Scalar(0, 255, 0), 2);
	    line(finalImage, Point(x, y), Point(x+25, y), Scalar(0, 255, 0), 2);
        ostringstream convert; //FIXME: make this a member variable
        convert << "tracking object at (" << x << ", " << y << ")";
        setCenterPoint(x, y); // Set member variable
	    putText(finalImage, convert.str(), Point(x, y), 1, 1, Scalar(255, 0, 0), 2);
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
    float probability = 0.0;

    //cling to contours
    int maxX = -1;
    int minX = 1000000;
    int maxY = -1;
    int minY = 1000000;
    const int OFFSET = 7; // final: 5
    
    for(size_t i = 0; i < movingObjectCoors.at(0).size(); i++)
    {   
        if((movingObjectCoors.at(0).at(i).y < centerPixel.y) && (i+OFFSET <= prevImage.rows)
        && (movingObjectCoors.at(0).at(i).x < centerPixel.x) && (i+OFFSET <= prevImage.cols) )
        {
            prevImage.at<cv::Vec3b>(movingObjectCoors.at(0).at(i).y + OFFSET, movingObjectCoors.at(0).at(i).x + OFFSET)[2] = 255;
        }
        else if((movingObjectCoors.at(0).at(i).y > centerPixel.y)  && (i+OFFSET <= prevImage.rows)
             && (movingObjectCoors.at(0).at(i).x < centerPixel.x) && (i+OFFSET <= prevImage.cols) )
        {
            prevImage.at<cv::Vec3b>(movingObjectCoors.at(0).at(i).y - OFFSET, movingObjectCoors.at(0).at(i).x + OFFSET)[2] = 255;
        }
        else if((movingObjectCoors.at(0).at(i).y < centerPixel.y)  && (i+OFFSET <= prevImage.rows)
             && (movingObjectCoors.at(0).at(i).x > centerPixel.x) && (i+OFFSET <= prevImage.cols) )
        {
            prevImage.at<cv::Vec3b>(movingObjectCoors.at(0).at(i).y + OFFSET, movingObjectCoors.at(0).at(i).x - OFFSET)[2] = 255;
        }
        else if((movingObjectCoors.at(0).at(i).y > centerPixel.y)  && (i+OFFSET <= prevImage.rows)
             && (movingObjectCoors.at(0).at(i).x > centerPixel.x) && (i+OFFSET <= prevImage.cols) )
        {
            prevImage.at<cv::Vec3b>(movingObjectCoors.at(0).at(i).y - OFFSET, movingObjectCoors.at(0).at(i).x - OFFSET)[2] = 255;
        }
        else
        {
            prevImage.at<cv::Vec3b>(movingObjectCoors.at(0).at(i).y, movingObjectCoors.at(0).at(i).x)[2] = 255;
        }
        
        //hSum += hsvImage.at<cv::Vec3b>(centerPixel.y + i, centerPixel.x)[0];   // H
        //sSum += hsvImage.at<cv::Vec3b>(centerPixel.y + i, centerPixel.x)[1];   // S
        //vSum += hsvImage.at<cv::Vec3b>(centerPixel.y + i, centerPixel.x)[2];   // V
        if(movingObjectCoors.at(0).at(i).x > maxX)
            maxX = movingObjectCoors.at(0).at(i).x;
        else if(movingObjectCoors.at(0).at(i).x < minX)
            minX = movingObjectCoors.at(0).at(i).x;

        
        if(movingObjectCoors.at(0).at(i).y > maxY)
            maxY = movingObjectCoors.at(0).at(i).y;
        else if(movingObjectCoors.at(0).at(i).y < minY)
            minY = movingObjectCoors.at(0).at(i).y;

    }


    if(((maxX - minX) < 25) && ((maxY - minY) < 25) ) // i.e. moving object is about the right size
        cv::rectangle(prevImage, Point(minX+OFFSET, minY+OFFSET), Point(maxX-OFFSET, maxY-OFFSET), Scalar(255, 0, 0), 1); // blue?
        
        
    float count = 0.0;
    for(size_t y = 0; y < maxY; y++)
    {
        for(size_t x = 0; x < maxX; x++)
        {
            hSum += hsvImage.at<cv::Vec3b>(y, x)[0]; // H
            sSum += hsvImage.at<cv::Vec3b>(y, x)[1]; // S
            vSum += hsvImage.at<cv::Vec3b>(y, x)[2]; // V
            count++;
        }
    }
    
    hAv = hSum / count;
    sAv = sSum / count;
    vAv = sSum / count;
    /*if((hAv > 69.0 && hAv < 80.0)
    && (sAv > 39.0 && sAv < 50.0)
    && (vAv > 39.0 && vAv < 50.0) )*/
        //cout << "Average HSV: (" << hAv << ", " << sAv << ", " << vAv << ")" << endl;


    //cv::imshow("Bound + 25px", prevImage);
    //cv::waitKey(3);

    return probability;
}


bool MotionSeg::closeEnough(int x, int y, geometry_msgs::Point theCenterPoint)
{
    if(theCenterPoint.x == -1)
    {
        return true;//first iteration of run
    }
    else if((x > (theCenterPoint.x-25)) && (x < (theCenterPoint.x+25))
         && (y > (theCenterPoint.y-25)) && (y < (theCenterPoint.y+25)) )
    {
        return true;
    }
    else
    {
        return false;
    }
}


bool MotionSeg::hsvExistsNear(Mat cvImage, geometry_msgs::Point centerPoint) //!!!implement!!!
{
    cv::Mat hsxImage;
    cv::cvtColor(cvImage, hsxImage, CV_BGR2HSV);
    cv::Mat inrangeImage;
    cv::inRange(hsxImage,
                Scalar(0, 0, 255 ),
                Scalar(116, 239, 361),
                inrangeImage);

    //cv::imshow("HSV Image", hsxImage);
    //cv::waitKey(3);
    //cv::imshow("inRange Image", inrangeImage);
    //cv::waitKey(3);
    
    bool objectDetected = false;
    Mat temp;
    inrangeImage.copyTo(temp);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    int theObject[2] = {0, 0};
    Rect objectBoundingRectangle = Rect(0, 0, 0, 0);

    findContours(temp, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    if(contours.size() > 0)
    {
        ROS_INFO("# of shapes found: %lu", contours.size() );
        objectDetected = true;
    }
    else
    {
        objectDetected = false;
    }
    
    //find objects of specified HSV values which are of a certain size
    for(size_t i = 0; i < contours.size(); i++)
    {
        if((contours.at(i).size() > 25) || (contours.at(i).size() < 3) )
        {
            contours.erase(contours.begin() );
        }
    }
    
    
    //compare center of each object with the centerPoint (from motion)
    for(size_t a = 0; a < contours.size(); a++)
    {
        geometry_msgs::Point tmpObjPoint;
        int xSum = 0;
        int ySum = 0;
        int count = 0;
        for(size_t b = 0; b < contours.at(a).size(); b++)
        {
            xSum += contours.at(a).at(b).x;
            ySum += contours.at(a).at(b).y;
            count++;
        }
        
        int xAv = xSum / count;
        int yAv = ySum / count;
        if(((xAv < (centerPoint.x + 25)) && (xAv > (centerPoint.x - 25)))
        && ((yAv < (centerPoint.y + 25)) && (yAv > (centerPoint.y - 25))) )
        {
            ;//found object near chosen motion object
        }
    }//end of loop
    
    
    
    return true;
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


string* toString()
{
    ;
}


MotionSeg::~MotionSeg()
{
    ;
}
