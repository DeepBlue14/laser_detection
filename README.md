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
rosrun laser_detection ColorSeg_main
rosrun laser_detection ShapeSeg_main
rosrun laser_detection MotionSeg_main

rosrun rqt_reconfigure rqt_reconfigure
rosrun rviz rviz
```

- In the rqt_reconfigure gui, select *camera* --> *driver* --> *depth_registration*
- In Rviz, select a "PointCloud2" option, and for the topic choose "/scooter/depth_registered/points".
- Select "GreenDetectionPcl" from the menu of the rqt_reconfigure gui.



