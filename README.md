##**Laser Detection**

![Logo](https://github.com/DeepBlue14/laser_detection/blob/master/laser.jpg)

**Index**
- Project Synopsis
- Dependencies
- Build & Run instructions


####**Project Synposis**

The purpose of this project is to focus on a given object which the laser is pointing at.  Data concerning the object (distance, etc) will be computed.


####**Dependencies**

- ROS
- PCL
- OpenCV


####**Build & Run**


**Run:**

To run individual nodes:
```bash
roslaunch openni2_launch openni2.launch
rosrun rviz rviz
rosrun laser_detection MotionSeg_main /*OR*/ rosrun laser_detection ClickedImg
rosrun laser_detection EvalPoint
rosrun rqt_reconfigure rqt_reconfigure
```

- In the rqt_reconfigure gui, select *camera* --> *driver* --> *depth_registration*
- In rqt_reconfigure, select the "activate" checkbox for each node
- In Rviz, select a "PointCloud2" option, and for the topic choose "/scooter/depth_registered/points".
- *(Optional)* You can change parameters of the detection and segmentation using rqt_reconfigure

**Notes:**
Topics are published to ```/scooter/*```

