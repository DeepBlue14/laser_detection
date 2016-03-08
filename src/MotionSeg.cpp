#include "MotionSeg.h"


MotionSeg::MotionSeg(int sensitivity, int blur) : SENSITIVITY(sensitivity), BLUR(blur)
{
    nextIterBool = false;
    isInitialized = false;
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

    if(nextIterBool == true && isInitialized == true)
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
    isInitialized = true;
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
    cv::threshold(differenceImage, thresholdImage, SENSITIVITY, 255, THRESH_BINARY);
    cv::blur(thresholdImage, thresholdImage, cv::Size(BLUR, BLUR) ); //FIXME: this breaks on start
    cv::threshold(thresholdImage, thresholdImage, SENSITIVITY, 255, THRESH_BINARY);

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
        //ROS_INFO("# of shapes found: %lu", contours.size() );
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
            //FIXME: this assumes that the last moving object in the vector is the correct one--an assumption that may not always be true
            //a better (but slower) way of doing this would be to send ALL of the moving objects to the verifyColor() filter, and then 
            //choose the best fit as the Chosen One.
            largestContourVec.push_back(contours.at(contours.size() - 1) ); 
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

    if(objectDetected)
    {
        //cout << "objectDetected = true" << endl;
        //draw crosshair scope in green
	    circle(finalImage, Point(x, y), 20, Scalar(0, 255, 0), 2);
	    line(finalImage, Point(x, y), Point(x, y-25), Scalar(0, 255, 0), 2);
	    line(finalImage, Point(x, y), Point(x, y+25), Scalar(0, 255, 0), 2);
	    line(finalImage, Point(x, y), Point(x-25, y), Scalar(0, 255, 0), 2);
	    line(finalImage, Point(x, y), Point(x+25, y), Scalar(0, 255, 0), 2);
        ostringstream convert; //TODO: make this a member variable
        convert << "tracking object at (" << x << ", " << y << ")";
        setCenterPoint(x, y); // Set member variable
	    putText(finalImage, convert.str(), Point(x, y), 1, 1, Scalar(255, 0, 0), 2);
    }
    
    cv::imshow("Final Image", finalImage);
    cv::waitKey(3);
}


bool MotionSeg::verifySize()
{
    
}


float MotionSeg::verifyColor(vector<vector<Point> > movingObjectCoors, Point centerPixel)
{
    float probability = 0.0;

    //cling to contours
    int maxX = -1;
    int minX = 1000000;
    int maxY = -1;
    int minY = 1000000;
    const int OFFSET = 7; // try 5?

    ///-----------------------------------------------------------
    //find rectangular bounds for laser, and then check to see
    //if the size and shape is valid.
    bool isTheLaser = false;
    for(size_t i = 0; i < movingObjectCoors.at(0).size(); i++)
    {   
        if(movingObjectCoors.at(0).at(i).x > maxX)
            maxX = movingObjectCoors.at(0).at(i).x;
        else if(movingObjectCoors.at(0).at(i).x < minX)
            minX = movingObjectCoors.at(0).at(i).x;

        
        if(movingObjectCoors.at(0).at(i).y > maxY)
            maxY = movingObjectCoors.at(0).at(i).y;
        else if(movingObjectCoors.at(0).at(i).y < minY)
            minY = movingObjectCoors.at(0).at(i).y;

    }

    if(( (maxX - minX) < 25) && ((maxX - minX) > 2) && ((maxY - minY) < 25) && ((maxY - minY) > 2) && (minX+OFFSET) > -1 && (maxX-OFFSET) > -1 && (minY+OFFSET) > -1 && (maxY-OFFSET) > -1)
    {
        isTheLaser = true;
        probability += 0.40;
    }
    //return isTheLaser;
    ///-----------------------------------------------------------

    if( ((maxX - minX) < 25) && ((maxX - minX) > 2) && ((maxY - minY) < 25) && ((maxY - minY) > 2) ) // i.e. moving object is about the right size
    {
        int averageRed = 0;
        int averageGreen = 0;
        int averageBlue = 0;
        int whiteCounter = 0;
        //calculate averate rgb values for potental laser

        if( (minX+OFFSET) > -1 && (maxX-OFFSET) > -1 && (minY+OFFSET) > -1 && (maxY-OFFSET) > -1)
        {
            for(size_t i = (minX+OFFSET); i < (maxX-OFFSET); i++)
            {
                for(size_t j = (minY+OFFSET); j < (maxY-OFFSET); j++)
                {
                    //cout << "HERE (13) (minY+OFFSET): " << (minY+OFFSET) << ", (maxY-OFFSET): " << (maxY-OFFSET) << endl;
                    averageRed += prevImage.at<cv::Vec3b>(j, i)[0]; //FIXME: found the seg fault, found the seg fault, found the seg fault!

                    averageGreen += prevImage.at<cv::Vec3b>(j, i)[1];
                    averageBlue += prevImage.at<cv::Vec3b>(j, i)[2];

                    //draw pixels blue
                    //FIXME: this is breaking whiteCounter because the pixels are now blue (instead of white)
                    //prevImage.at<cv::Vec3b>(j, i)[0] = 255;
                    //prevImage.at<cv::Vec3b>(j, i)[1] = 140;
                    //prevImage.at<cv::Vec3b>(j, i)[2] = 0;

                
                   if(prevImage.at<cv::Vec3b>(j, i)[0] > 250 && 
                    prevImage.at<cv::Vec3b>(j, i)[1] > 250 && 
                    prevImage.at<cv::Vec3b>(j, i)[0] > 250)
                   {
                        cout << "wc" << whiteCounter << endl;
                        whiteCounter++;
                   }
                } 
            }
        }
        

        int totalCount = (((maxX-OFFSET)-(minX+OFFSET)) * ((maxY-OFFSET)-(minY+OFFSET)));

        if(totalCount > 0)
        {
            averageRed = averageRed/totalCount;
            averageGreen = averageGreen/totalCount;
            averageBlue = averageBlue/totalCount;
        }
        
        //cout << "average: rgb(" << averageRed << ", " << averageGreen << ", " << averageBlue << ")" << endl;
        
        //we have a winner (size-wise)! 
        //draw pink rectangle around object
        cv::rectangle(prevImage, Point(minX+OFFSET, minY+OFFSET), Point(maxX-OFFSET, maxY-OFFSET), Scalar(255, 0, 255), 1);
        cv::rectangle(prevImage, Point(minX+OFFSET-10, minY+OFFSET-10), Point(maxX-OFFSET+10, maxY-OFFSET+10), Scalar(0, 0, 255), 1);
        //TODO scan the larger rectangle and sample each pixel in it for green > red && green > blue
        //this if true it will increase confidence of laser
        cout << "whitecounter: " << whiteCounter << endl;
        if(whiteCounter >= 5)//old: 10
        {
            cout << "\033[0;34m.................\033[0m\n" << endl;
            cout << "\033[0;34m...found laser...\033[0m\n" << endl;
            //cout << "\033[0;34m(" << (minX+maxX/2) << ", " << (minY+maxY/2) << ")\033[0m\n" << endl;
            cout << "\033[0;34m.................\033[0m\n" << endl;
            probability += 0.40;
        }
    }   



    cv::imshow("Bound + 25px", prevImage);
    cv::waitKey(3);

    cout << "Confidence: " << probability*100 << "%" << endl;
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
        //ROS_INFO("# of shapes found: %lu", contours.size() );
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
