#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from moveit_commander import MoveGroupCommander
import moveit_commander

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_xr2_ik", anonymous=True)
gripper = moveit_commander.MoveGroupCommander('claw_group')
reference_frame = 'base_link'
gripper.set_goal_joint_tolerance(0.1)
gripper.set_pose_reference_frame(reference_frame)
joint_positions = [0.0,-0.0]
gripper.set_joint_value_target(joint_positions)
gripper.go()
rospy.sleep(5)
joint_positions = [0.7,-0.7]
gripper.set_joint_value_target(joint_positions)
gripper.go()
rospy.sleep(5)
