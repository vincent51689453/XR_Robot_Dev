# XR_Robot_Dev
This repository is to develop XR Robot control with ROS melodic on Jetson TX2.

**1. Startup sequence**
---------------------------
### 1.1 Connect XR-Robot
```
$ roslaunch xrrobot Arduino.launch
```
![image](https://github.com/vincent51689453/XR_Robot_Dev/blob/master/git_image/connect_ok.png)


### 1.2 Rviz control demo
```
$ roslaunch arm_desktop_A1 display.launch
```
![image](https://github.com/vincent51689453/XR_Robot_Dev/blob/master/git_image/robot_arm.png)

![image](https://github.com/vincent51689453/XR_Robot_Dev/blob/master/git_image/rviz_control.gif)

### 1.3 Moiveit control demo
```
$ roslaunch arm_A1_Moveit demo.launch
```
![image](https://github.com/vincent51689453/XR_Robot_Dev/blob/master/git_image/moveit.png)


