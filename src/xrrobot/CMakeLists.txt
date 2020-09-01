cmake_minimum_required(VERSION 2.8.3)
project(xrrobot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosserial_python
  sensor_msgs
  std_msgs
  tf
)
catkin_package(
  LIBRARIES xrrobot
  CATKIN_DEPENDS roscpp rospy tf   geometry_msgs sensor_msgs std_msgs 
  DEPENDS system_lib
)

###########
## Build ##
###########




