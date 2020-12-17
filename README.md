# XR_Robot_ikfast
This repository is to develop XR Robot control with ROS melodic on Jetson TX2 with ikfast .

**1. Startup**
---------------------------
### 1.1 Connect XR-Robot
```
$ sudo chmod 666 /dev/ttyUSB0
$ roslaunch xrrobot Arduino.launch
```
![image](https://github.com/vincent51689453/XR_Robot_Dev/blob/master/git_image/connect_ok.png)


### 1.2 Rviz control demo
This demo can show joint by joint adjustment of xrrobot
```
$ roslaunch myarm_a1 display.launch
```
![image](https://github.com/vincent51689453/XR_Robot_Dev/blob/master/git_image/robot_arm.png)

![image](https://github.com/vincent51689453/XR_Robot_Dev/blob/master/git_image/rviz_control.gif)

### 1.3 Moiveit control demo
This demo can show the capabilitiy and reaction of manual assigned destination.
```
$ roslaunch myarm_a1_moveit_config demo.launch
```
![image](https://github.com/vincent51689453/XR_Robot_Dev/blob/master/git_image/moveit.png)


### 1.4 Python control using ikfast
This demo can demonstrate a series of action "pick and release actions" by python control.
```
$ sudo chmod a+x src/xrrobot_control/src/robot_demo.py
$ roslaunch myarm_a1_moveit_config demo.launch
$ rosrun xrrobot_control robot_demo.py
```


