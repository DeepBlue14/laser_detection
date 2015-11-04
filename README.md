##**Laser Detection**

![Logo](https://github.com/DeepBlue14/laser_detection/blob/master/laser.jpg)

**Index**
- Project Synopsis
- Dependencies
- Build & Run instructions


####**Project Synposis**

The purpose of this project is to focus on a given object which the laser is pointing at.  Data concerning the object (distance, etc) will be computed.

**Nodes**
- MotionSeg_main

This node uses a combination of shape, HSV, and motion to detect the laser point.  It publishes to geometry_msgs::Point.  The point represents the (x, y) pixel location on the image where the center of the laser dot was located.  It will stop publishing when it did not detect movement; i.e. the last published msg should represent the location of the laser after the user stopped moving it.  It publishes to ```/scooter/geometry_msgs/center_point```.

- ClickedImg

This node allows the user to click on the image being published from the sensor (click on the **Final Image** frame, NOT the **Initial Image**).  This point is published to ```/scooter/geometry_msgs/center_point```. 

- EvalPoint

This node subscribes subscribes to ```/scooter/geometry_msgs/center_point``` (as well as image and pc topics).  It performs background subtraction and point clustering.  Next, it compares the pixel point with the clusters to determine which one was clicked.  To view the result, subscribe to ```/scooter/rgb/image_with_box``` in rviz (pointcloud2 type).  Clusters will be represented in yellow; the selected object will be green if in range (currently set at 1 meter) or red if out of range.



####**Dependencies**

- ROS **(tested on indigo)**
- PCL
- OpenCV
**Warning:**
**OpenCV 3.0 for Linux has a known bug where the cv::imshow method may seg fault.  Therefore, I discourage using this version (as this program does use cv::imshow).  If 3.0 is nessisary, I can take out the imshow and replace it with image_view :(**


####**Build & Run**


**Run:**

To run individual nodes:
```bash
roslaunch openni2_launch openni2.launch
rosrun rviz rviz
rosrun object_separator object_separator #catkin_ws
rosrun laser_detection MotionSeg_main
```



