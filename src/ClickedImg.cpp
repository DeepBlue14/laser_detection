#include "ClickedImg.h"


void mouseCallback(int event, int x, int y, int flags, void* userdata)
{
cout << "mouseCallback called" << endl;
    switch(event)
    {
        case EVENT_FLAG_LBUTTON:
            cout << "Position (" << x << ", " << y << ")" << endl;
            //drawBoundingBox(x, y, 0, 0);
            break;
        case CV_EVENT_MOUSEMOVE:
        cout << "testing" << endl;
        //drawBoundingBox(x, y, 0, 0);
        break;
    } 

}


ClickedImg::ClickedImg()
{
    pub = new Publisher();
    centerPoint.z = -1;
    namedWindow("Result", 1);
    setMouseCallback("Result", mouseCallback, NULL);
}


void ClickedImg::callback(const sensor_msgs::ImageConstPtr& input)
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

    cv::imshow("Initial Image", cvImage);
    cv::waitKey(3);

    if(nextIterBool == true)
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

    //pub->publish(cv_ptr->toImageMsg() );
    pub->publish(centerPoint);
}


void ClickedImg::filterByMotion(Mat nextImage)
{
    Mat grayImage1;
    Mat grayImage2;
    Mat differenceImage;
    Mat thresholdImage;
/*
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
    cv::waitKey(3);*/
    
    rectangle(nextImage, Point(25,25), Point(100,100), Scalar(0,255,0), 2);
    imshow("Final Image", nextImage);
    cv::waitKey(3);
}


void ClickedImg::setCenterPoint(int x, int y)
{
    centerPoint.x = x;
    centerPoint.y = y;
}


geometry_msgs::Point ClickedImg::getCenterPoint()
{
    return centerPoint;
}


Publisher* ClickedImg::getPublisher()
{
    return pub;
}


ClickedImg::~ClickedImg()
{
    ;
}
