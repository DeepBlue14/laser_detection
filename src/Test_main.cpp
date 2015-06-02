#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;
using namespace cv;

//our sensitivity value to be used in the threshold() function
const static int SENSITIVITY_VALUE = 20;
//size of blur used to smooth the image to remove possible noise and
//increase the size of the object we are trying to track. (Much like dilate and erode)
const static int BLUR_SIZE = 10;
//we'll have just one object to search for
//and keep track of its position.
int theObject[2] = {0,0};
//bounding rectangle of the object, we will use the center of this as its position.
Rect objectBoundingRectangle = Rect(0,0,0,0);


//int to string helper function
string intToString(int number){

	//this function has a number input and string output
	std::stringstream ss;
	ss << number;
	return ss.str();
}

void searchForMovement(Mat thresholdImage, Mat &cameraFeed){
	//notice how we use the '&' operator for the cameraFeed. This is because we wish
	//to take the values passed into the function and manipulate them, rather than just working with a copy.
	//eg. we draw to the cameraFeed in this function which is then displayed in the main() function.
	bool objectDetected=false;
	Mat temp;
	thresholdImage.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	//findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );// retrieves all contours
	findContours(temp,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );// retrieves external contours

	//if contours vector is not empty, we have found some objects
	if(contours.size()>0)objectDetected=true;
	else objectDetected = false;

	if(objectDetected){
		//the largest contour is found at the end of the contours vector
		//we will simply assume that the biggest contour is the object we are looking for.
		vector< vector<Point> > largestContourVec;
		largestContourVec.push_back(contours.at(contours.size()-1));
		//make a bounding rectangle around the largest contour then find its centroid
		//this will be the object's final estimated position.
		objectBoundingRectangle = boundingRect(largestContourVec.at(0));
		int xpos = objectBoundingRectangle.x+objectBoundingRectangle.width/2;
		int ypos = objectBoundingRectangle.y+objectBoundingRectangle.height/2;

		//update the objects positions by changing the 'theObject' array values
		theObject[0] = xpos , theObject[1] = ypos;
	}
	//make some temp x and y variables so we dont have to type out so much
	int x = theObject[0];
	int y = theObject[1];
	//draw some crosshairs on the object
	circle(cameraFeed,Point(x,y),20,Scalar(0,255,0),2);
	line(cameraFeed,Point(x,y),Point(x,y-25),Scalar(0,255,0),2);
	line(cameraFeed,Point(x,y),Point(x,y+25),Scalar(0,255,0),2);
	line(cameraFeed,Point(x,y),Point(x-25,y),Scalar(0,255,0),2);
	line(cameraFeed,Point(x,y),Point(x+25,y),Scalar(0,255,0),2);
	putText(cameraFeed,"Tracking object at (" + intToString(x)+","+intToString(y)+")",Point(x,y),1,1,Scalar(255,0,0),2);



}
int main(){

	//some boolean variables for added functionality
	bool objectDetected = false;
	//these two can be toggled by pressing 'd' or 't'
	bool debugMode = false;
	bool trackingEnabled = false;
	//pause and resume code
	bool pause = false;
	//set up the matrices that we will need
	//the two frames we will be comparing
	Mat frame1,frame2;
	//their grayscale images (needed for absdiff() function)
	Mat grayImage1,grayImage2;
	//resulting difference image
	Mat differenceImage;
	//thresholded difference image (for use in findContours() function)
	Mat thresholdImage;
	//video capture object.
	VideoCapture capture;

	while(1){

		//we can loop the video by re-opening the capture every time the video reaches its last frame

		capture.open("bouncingBall.avi");

		if(!capture.isOpened()){
			cout<<"ERROR ACQUIRING VIDEO FEED\n";
			getchar();
			return -1;
		}

		//check if the video has reach its last frame.
		//we add '-1' because we are reading two frames from the video at a time.
		//if this is not included, we get a memory error!
		while(capture.get(CV_CAP_PROP_POS_FRAMES)<capture.get(CV_CAP_PROP_FRAME_COUNT)-1){

			//read first frame
			capture.read(frame1);
			//convert frame1 to gray scale for frame differencing
            cv::cvtColor(frame1, grayImage1, COLOR_BGR2GRAY);
			//copy second frame
            capture.read(frame2);
			//convert frame2 to gray scale for frame differencing
            cv::cvtColor(frame2, grayImage2, COLOR_BGR2GRAY);
			//perform frame differencing with the sequential images. This will output an "intensity image"
			//do not confuse this with a threshold image, we will need to perform thresholding afterwards.
            cv::absdiff(grayImage1, grayImage2, differenceImage);
			//threshold intensity image at a given sensitivity value
            cv::threshold(differenceImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);//***change sensitivity value***?
			if(debugMode==true){
				//show the difference image and threshold image
                cv::imshow("Difference Image", differenceImage);
                cv::imshow("Threshold Image", thresholdImage);
			}else{
				//if not in debug mode, destroy the windows so we don't see them anymore
                cv::destroyWindow("Difference Image");
                cv::destroyWindow("Threshold Image");
			}
			//use blur() to smooth the image, remove possible noise and
			//increase the size of the object we are trying to track. (Much like dilate and erode)
            cv::blur(thresholdImage, thresholdImage, cv::Size(BLUR_SIZE, BLUR_SIZE) );
			//threshold again to obtain binary image from blur output
            cv::threshold(thresholdImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);
			if(debugMode==true){
				//show the threshold image after it's been "blurred"
                
                imshow("Final Threshold Image", thresholdImage);

			}
			else {
				//if not in debug mode, destroy the windows so we don't see them anymore
                cv::destroyWindow("Final Threshold Image");
			}

			//if tracking enabled, search for contours in our thresholded image
            if(trackingEnabled){
            
                searchForMovement(thresholdImage, frame1);
            }
            
            
			//show our captured frame
			imshow("Frame1",frame1);
			//check to see if a button has been pressed.
			//this 10ms delay is necessary for proper operation of this program
			//if removed, frames will not have enough time to referesh and a blank 
			//image will appear.
			switch(waitKey(10)){

			case 27: //'esc' key has been pressed, exit program.
				return 0;
			case 116: //'t' has been pressed. this will toggle tracking
				trackingEnabled = !trackingEnabled;
				if(trackingEnabled == false) cout<<"Tracking disabled."<<endl;
				else cout<<"Tracking enabled."<<endl;
				break;
			case 100: //'d' has been pressed. this will debug mode
				debugMode = !debugMode;
				if(debugMode == false) cout<<"Debug mode disabled."<<endl;
				else cout<<"Debug mode enabled."<<endl;
				break;
			case 112: //'p' has been pressed. this will pause/resume the code.
				pause = !pause;
				if(pause == true){ cout<<"Code paused, press 'p' again to resume"<<endl;
				while (pause == true){
					//stay in this loop until 
					switch (waitKey()){
						//a switch statement inside a switch statement? Mind blown.
					case 112: 
						//change pause back to false
						pause = false;
						cout<<"Code resumed."<<endl;
						break;
					}
				}
				}


			}


		}
		//release the capture before re-opening and looping again.
		capture.release();
	}

	return 0;

}




//------------------------------------------------------------------------------------------------------



/*
//https://blog.cedric.ws/opencv-simple-motion-detection
//https://github.com/cedricve/motion-detection/blob/master/motion_src/CMakeLists.txt

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;
using namespace cv;

// Check if the directory exists, if not create it
// This function will create a new directory if the image is the first
// image taken for a specific day
inline void directoryExistsOrCreate(const char* pzPath)
{
    DIR *pDir;
    // directory doesn't exists -> create it
    if ( pzPath == NULL || (pDir = opendir (pzPath)) == NULL)
        mkdir(pzPath, 0777);
    // if directory exists we opened it and we
    // have to close the directory again.
    else if(pDir != NULL)
        (void) closedir (pDir);
}

// When motion is detected we write the image to disk
//    - Check if the directory exists where the image will be stored.
//    - Build the directory and image names.
int incr = 0;
inline bool saveImg(Mat image, const string DIRECTORY, const string EXTENSION, const char * DIR_FORMAT, const char * FILE_FORMAT)
{
    stringstream ss;
    time_t seconds;
    struct tm * timeinfo;
    char TIME[80];
    time (&seconds);
    // Get the current time
    timeinfo = localtime (&seconds);

    // Create name for the date directory
    strftime (TIME,80,DIR_FORMAT,timeinfo);
    ss.str("");
    ss << DIRECTORY << TIME;
    directoryExistsOrCreate(ss.str().c_str());
    ss << "/cropped";
    directoryExistsOrCreate(ss.str().c_str());

    // Create name for the image
    strftime (TIME,80,FILE_FORMAT,timeinfo);
    ss.str("");
    if(incr < 100) incr++; // quick fix for when delay < 1s && > 10ms, (when delay <= 10ms, images are overwritten)
    else incr = 0;
    ss << DIRECTORY << TIME << static_cast(incr) << EXTENSION;
    return imwrite(ss.str().c_str(), image);
}

// Check if there is motion in the result matrix
// count the number of changes and return.
inline int detectMotion(const Mat & motion, Mat & result, Mat & result_cropped,
                 int x_start, int x_stop, int y_start, int y_stop,
                 int max_deviation,
                 Scalar & color)
{
    // calculate the standard deviation
    Scalar mean, stddev;
    meanStdDev(motion, mean, stddev);
    // if not to much changes then the motion is real (neglect agressive snow, temporary sunlight)
    if(stddev[0] < max_deviation)
    {
        int number_of_changes = 0;
        int min_x = motion.cols, max_x = 0;
        int min_y = motion.rows, max_y = 0;
        // loop over image and detect changes
        for(int j = y_start; j < y_stop; j+=2){ // height
            for(int i = x_start; i < x_stop; i+=2){ // width
                // check if at pixel (j,i) intensity is equal to 255
                // this means that the pixel is different in the sequence
                // of images (prev_frame, current_frame, next_frame)
                if(static_cast(motion.at(j,i)) == 255)
                {
                    number_of_changes++;
                    if(min_x>i) min_x = i;
                    if(max_xj) min_y = j;
                    if(max_y 0) min_x -= 10;
            if(min_y-10 > 0) min_y -= 10;
            if(max_x+10 < result.cols-1) max_x += 10;
            if(max_y+10 < result.rows-1) max_y += 10;
            // draw rectangle round the changed pixel
            Point x(min_x,min_y);
            Point y(max_x,max_y);
            Rect rect(x,y);
            Mat cropped = result(rect);
            cropped.copyTo(result_cropped);
            rectangle(result,rect,color,1);
        }
        return number_of_changes;
    }
    return 0;
}

int main (int argc, char * const argv[])
{
    const string DIR = "/home/pi/motion_src/pics/"; // directory where the images will be stored
    const string EXT = ".jpg"; // extension of the images
    const int DELAY = 500; // in mseconds, take a picture every 1/2 second
    const string LOGFILE = "/home/pi/motion_src/log";

    // Format of directory
    string DIR_FORMAT = "%d%h%Y"; // 1Jan1970
    string FILE_FORMAT = DIR_FORMAT + "/" + "%d%h%Y_%H%M%S"; // 1Jan1970/1Jan1970_12153
    string CROPPED_FILE_FORMAT = DIR_FORMAT + "/cropped/" + "%d%h%Y_%H%M%S"; // 1Jan1970/cropped/1Jan1970_121539

    // Set up camera
    CvCapture * camera = cvCaptureFromCAM(CV_CAP_ANY);
    cvSetCaptureProperty(camera, CV_CAP_PROP_FRAME_WIDTH, 1280); // width of viewport of camera
    cvSetCaptureProperty(camera, CV_CAP_PROP_FRAME_HEIGHT, 720); // height of ...

    // Take images and convert them to gray
    Mat result, result_cropped;
    Mat prev_frame = result = cvQueryFrame(camera);
    Mat current_frame = cvQueryFrame(camera);
    Mat next_frame = cvQueryFrame(camera);
    cvtColor(current_frame, current_frame, CV_RGB2GRAY);
    cvtColor(prev_frame, prev_frame, CV_RGB2GRAY);
    cvtColor(next_frame, next_frame, CV_RGB2GRAY);

    // d1 and d2 for calculating the differences
    // result, the result of and operation, calculated on d1 and d2
    // number_of_changes, the amount of changes in the result matrix.
    // color, the color for drawing the rectangle when something has changed.
    Mat d1, d2, motion;
    int number_of_changes, number_of_sequence = 0;
    Scalar mean_, color(0,255,255); // yellow

    // Detect motion in window
    int x_start = 10, x_stop = current_frame.cols-11;
    int y_start = 350, y_stop = 530;

    // If more than 'there_is_motion' pixels are changed, we say there is motion
    // and store an image on disk
    int there_is_motion = 5;

    // Maximum deviation of the image, the higher the value, the more motion is allowed
    int max_deviation = 20;

    // Erode kernel
    Mat kernel_ero = getStructuringElement(MORPH_RECT, Size(2,2));

    // All settings have been set, now go in endless loop and
    // take as many pictures you want..
    while (true){
        // Take a new image
        prev_frame = current_frame;
        current_frame = next_frame;
        next_frame = cvQueryFrame(camera);
        result = next_frame;
        cvtColor(next_frame, next_frame, CV_RGB2GRAY);

        // Calc differences between the images and do AND-operation
        // threshold image, low differences are ignored (ex. contrast change due to sunlight)
        absdiff(prev_frame, next_frame, d1);
        absdiff(next_frame, current_frame, d2);
        bitwise_and(d1, d2, motion);
        threshold(motion, motion, 35, 255, CV_THRESH_BINARY);
        erode(motion, motion, kernel_ero);

        number_of_changes = detectMotion(motion, result, result_cropped,  x_start, x_stop, y_start, y_stop, max_deviation, color);

        // If a lot of changes happened, we assume something changed.
        if(number_of_changes>=there_is_motion)
        {
            if(number_of_sequence>0){ 
                saveImg(result,DIR,EXT,DIR_FORMAT.c_str(),FILE_FORMAT.c_str());
                saveImg(result_cropped,DIR,EXT,DIR_FORMAT.c_str(),CROPPED_FILE_FORMAT.c_str());
            }
            number_of_sequence++;
        }
        else
        {
            number_of_sequence = 0;
            // Delay, wait a 1/2 second.
            cvWaitKey (DELAY);
        }
    }
    return 0;    
}
*/
//------------------------------------------------------------------------------------------------------
/*
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

using namespace std;
using namespace cv;

int main( int argc, char** argv )
{
  Mat src = imread( argv[1], 1 );
  Mat samples(src.rows * src.cols, 3, CV_32F);
  for( int y = 0; y < src.rows; y++ )
    for( int x = 0; x < src.cols; x++ )
      for( int z = 0; z < 3; z++)
        samples.at<float>(y + x*src.rows, z) = src.at<Vec3b>(y,x)[z];


  int clusterCount = 15;
  Mat labels;
  int attempts = 5;
  Mat centers;
  kmeans(samples, clusterCount, labels, TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10000, 0.0001), attempts, KMEANS_PP_CENTERS, centers );


  Mat new_image( src.size(), src.type() );
  for( int y = 0; y < src.rows; y++ )
    for( int x = 0; x < src.cols; x++ )
    { 
      int cluster_idx = labels.at<int>(y + x*src.rows,0);
      new_image.at<Vec3b>(y,x)[0] = centers.at<float>(cluster_idx, 0);
      new_image.at<Vec3b>(y,x)[1] = centers.at<float>(cluster_idx, 1);
      new_image.at<Vec3b>(y,x)[2] = centers.at<float>(cluster_idx, 2);
    }
    
  imshow( "clustered image", new_image );
  waitKey( 0 );
}
*/
//-----------------------------------------------------------------------------------------------------------
/*
//*** HSV segmentation ***
#include <ros/ros.h>
#include <ros/console.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
//#include <Windows.h>

using namespace std;

int main() {
    cv::VideoCapture capWebcam(0);

// get screen resolution, which will be used in future
        //int widthx = GetSystemMetrics(SM_CXSCREEN);
        //int heighty = GetSystemMetrics(SM_CYSCREEN);
        //std::cout << (widthx) << " X " << (heighty) << "\n\n";

//  webcam access
    if (capWebcam.isOpened() == false) {
        std::cout << "Error: Webcam don't work or can't be accessed\n\n";
        return(1);
    }

// creating various necessary staff
    cv::Mat matOriginal;
    cv::Mat matOriginalHSV;
    cv::Mat matProcessed;

    std::vector<cv::Vec3f> vecCircles;
    std::vector<cv::Vec3f>::iterator itrCircles;

    char charEscKey = 0;

    cv::namedWindow("Original", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("Processed", CV_WINDOW_AUTOSIZE);

// creating trackbars
    cv::namedWindow("Trackbar", CV_WINDOW_AUTOSIZE);
    int hmin = 0, smin = 0, vmin = 0,
        hmax = 180, smax = 256, vmax = 256;
    cv::createTrackbar("H min:", "Trackbar", &hmin, hmax);
    cv::createTrackbar("H max:", "Trackbar", &hmax, hmax);
    cv::createTrackbar("S min:", "Trackbar", &smin, smax);
    cv::createTrackbar("S max:", "Trackbar", &smax, smax);
    cv::createTrackbar("V min:", "Trackbar", &vmin, vmax);
    cv::createTrackbar("V max:", "Trackbar", &vmax, vmax);


// main function
    while (charEscKey != 27) {
        if (capWebcam.read(matOriginal) == NULL) {
            std::cout << "Error: frame not readed from Cam\n\n";
            break;
        }

// converting to HSV
        cv::cvtColor(matOriginal, matOriginalHSV, CV_BGR2HSV);

// spliting channels, changing any channel from "matOriginalHSVchannels[0/1/2]" to "matZero" to remove it
        cv::Mat matZero;
        cv::Mat matOriginalHS;
        matZero = cv::Mat::zeros(cv::Size(matOriginalHSV.cols, matOriginalHSV.rows), CV_8UC1);
        cv::vector<cv::Mat> matOriginalHSVchannels(3);
        cv::split(matOriginalHSV, matOriginalHSVchannels);
        cv::vector<cv::Mat> channels;
        channels.push_back(matOriginalHSVchannels[0]);
        channels.push_back(matOriginalHSVchannels[1]);
        channels.push_back(matZero);
        cv::merge(channels, matOriginalHS);

// search of laser spot
        cv::inRange(matOriginalHS, cv::Scalar(hmin, smin, vmin), cv::Scalar(hmax, smax, vmax), matProcessed);

// blur
        cv::GaussianBlur(matProcessed,
            matProcessed,
            cv::Size(5, 5),
            2.2);

// circles search 
        cv::HoughCircles(matProcessed,
            vecCircles,
            CV_HOUGH_GRADIENT,
            1,
            matProcessed.rows / 8,
            100,
            50,
            7,
            20);

// draw circles
        for (itrCircles = vecCircles.begin(); itrCircles != vecCircles.end(); itrCircles++) {
            std::cout << "position x = " << (*itrCircles)[0]
                << ", y = " << (*itrCircles)[1]
                << ", z = " << (*itrCircles)[2] << "\n";

            cv::circle(matOriginalHS,
                cv::Point((int)(*itrCircles)[0], (int)(*itrCircles)[1]),
                1,
                cv::Scalar(255, 0, 0),
                CV_FILLED);

            cv::circle(matOriginalHS,
                cv::Point((int)(*itrCircles)[0], (int)(*itrCircles)[1]),
                (int)(*itrCircles)[2],
                cv::Scalar(255, 255, 0),
                2);

// set mouse cursor on laser coordinates
            int screenx = (int)((*itrCircles)[0] * 2.2);
            int screeny = (int)((*itrCircles)[1] * 1.2);
           
            //SetCursorPos(screenx, screeny);

        }
        ROS_INFO("# of circles found: %lu", vecCircles.size() ); 
        cv::Mat matProcessedInv;
        cv::bitwise_not(matProcessed, matProcessedInv);
        
// show windows
        cv::imshow("Original", matOriginalHS);
        //cv::imshow("Processed", matProcessed);
        cv::imshow("Processed", matProcessedInv);
        charEscKey = cv::waitKey(10);
    }
    return(0);
}
*/
