/*
 * File: ClickedImg_main.cpp
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: 
 *
 * 
 */
 
 
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/Point.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <cstdlib>

using namespace ros;
using namespace pcl;
using namespace std;
using namespace cv;



cv::Mat cvImage;
geometry_msgs::Point clickedPoint;
Publisher* pub = new Publisher();



void mouseCallback(int event, int x, int y, int flags, void* userdata)
{
    //cout << "mouseCallback called" << endl;
    switch(event)
    {
        case EVENT_FLAG_LBUTTON:
            //drawBoundingBox(x, y, 0, 0);
            clickedPoint.x = x;
            clickedPoint.y = y;
            clickedPoint.z = -1;
            //cout << "Position (" << x << ", " << y << ")" << endl;
            pub->publish(clickedPoint);
            //cout << "Position (" << clickedPoint.x << ", " << clickedPoint.y << ")" << endl;
            break;
        case CV_EVENT_MOUSEMOVE:
        //cout << "testing" << endl;
        //drawBoundingBox(x, y, 0, 0);
        break;
    } 

}


void callback(const sensor_msgs::ImageConstPtr& input)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(input);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what() );
    }
    
    cvImage = cv_ptr->image;

    cv::imshow("Initial Image", cvImage);
    cv::waitKey(3);
    


    Mat t = cvImage;
    //- - - - - - - -
    //prevImage = cvImage;

    cv_ptr->image = cvImage;

    //pub->publish(cv_ptr->toImageMsg() );
    
    clickedPoint.x = -1;
    clickedPoint.y = -1;
    clickedPoint.z = -1;
    
    cv::imshow("Result", cvImage);
    cv::waitKey(3); 

    pub->publish(clickedPoint);
}


int main(int argc, char **argv)
{
    init(argc, argv, "ClickedImgmentation");
    
    ROS_INFO("Starting node\n");

    NodeHandle nh;
    Publisher* mainsPub = pub; 
    namedWindow("Result", 1);
    setMouseCallback("Result", mouseCallback, NULL);

    Subscriber sub = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color",
                                                        10,
                                                        callback);
                                                        

    *mainsPub = nh.advertise<geometry_msgs::Point>("/scooter/geometry_msgs/center_point", 10);


    spin();
    
    return EXIT_SUCCESS;
}








/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Reference: http://opencv-srf.blogspot.com/2011/11/mouse-events.html
//	     http://docs.opencv.org/modules/highgui/doc/qt_new_functions.html#createbutton
/*
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

using namespace std;
using namespace cv;
Mat img;

void drawBoundingBox(int x, int y, int width, int height)
{
    rectangle(img, Point(x,y), Point(x+20,y-20), Scalar(0,255,0), 2);
    imshow("My 2Win", img);
    waitKey(0);
}

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    switch(event)
    {
        case EVENT_FLAG_LBUTTON:
            cout << "Position (" << x << ", " << y << ")" << endl;
            drawBoundingBox(x, y, 0, 0);
            break;
        case CV_EVENT_MOUSEMOVE:
        cout << "testing" << endl;
        //drawBoundingBox(x, y, 0, 0);
        break;
    } 

}

int main(int argc, char** argv)
{
     // Read image from file 
     img = imread("/home/james/catkin_ws/src/rcv_image_view/test.jpg");

      //if fail to read the image
     if ( img.empty() ) 
     { 
          cout << "Error loading the image" << endl;
          return -1; 
     }

      //Create a window
     namedWindow("My Window", 1);

      //set the callback function for any mouse event
     setMouseCallback("My Window", CallBackFunc, NULL);

      //show the image
     imshow("My Window", img);
     waitKey(0);

     return 0;
}
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
