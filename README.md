##**Laser Detection**

![Logo](https://github.com/DeepBlue14/laser_detection/blob/master/laser.jpg)

**Index**
-Project Synopsis
- Build & Run instructions


####**Project Synposis**

...


####**Build & Run**

...

**Run:**
```bash
roslaunch openni2_launch openni2.launch
rosrun laser_detection GreenDetectionPcl_main
rosrun rqt_reconfigure rqt_reconfigure
rosrun rviz rviz
```

In Rviz, select a "PointCloud2" option, and for the topic choose "/frankenscooter/camera/points".
Next, select "GreenDetectionPcl" from the menu of the rqt_reconfigure gui.
