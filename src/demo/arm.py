#!/usr/bin/env python

import rospy, sys
import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from moveit_commander import MoveGroupCommander
import moveit_commander

rospy.init_node('servo_sub', anonymous=False)
pub = rospy.Publisher('move_group/fake_controller_joint_states', JointState, queue_size=10)
moveit_commander.roscpp_initialize(sys.argv)
gripper = moveit_commander.MoveGroupCommander('claw_group')
servo_str = JointState()
gripper.go()
servo_str.name = ['f']
servo_str.position = [0.7]
pub.publish(servo_str)
time.sleep(1)


