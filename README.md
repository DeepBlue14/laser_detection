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
```bash
roslaunch openni2_launch openni2.launch
rosrun laser_detection GreenDetectionCv_main
rosrun laser_detection GreenPart2_main
rosrun rqt_reconfigure rqt_reconfigure
rosrun rviz rviz
```

- In the rqt_reconfigure gui, select *camera* --> *driver* --> *depth_registration*
- In Rviz, select a "PointCloud2" option, and for the topic choose "/scooter/depth_registered/points".
- Select "GreenDetectionPcl" from the menu of the rqt_reconfigure gui.

**NOTE**: If GreenPart2_main segfaults when you begin it, rosrun it again. *(This error occurs if two threads are out of sync.  A fix for this is being worked in pressently.)*
