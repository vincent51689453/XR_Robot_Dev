#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    : 20-8-1 下午10:02
# @Author  : WX1995
# @Site    : 
# @File    : moveit_cartesian_demo.py
# @Software: PyCharm
#抓手垂直于地面四元数是: 0.491


import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy


class MoveitPickPlaceDemo:
		
	def __init__(self):
		self.group_name_gripper = "claw_group"
		self.gripper_reference  = "grasping_frame"
		
		# 初始化需要使用move group控制的机械臂中的gripper group
		self.gripper = MoveGroupCommander(self.group_name_gripper)

		# 设置允许的最大速度和加速度
        	self.gripper.set_max_acceleration_scaling_factor(0.03)
        	self.gripper.set_max_velocity_scaling_factor(0.05)
		
		#设置目标位置所使用的参考系
		self.gripper.set_pose_reference_frame(self.gripper_reference)		

		# 获得终端link
		self.end_effect_link = self.gripper.get_end_effector_link()
		print(self.end_effect_link)
		print(self.gripper)
				

		#控制夹爪处于正常状态
		self.gripper.set_named_target("open_claw")
		rospy.loginfo("init claw")
		self.gripper.go()
		rospy.sleep(1)


	def pick_objects(self):
		self.gripper.set_named_target("close_claw")
		rospy.loginfo("start pick!")
		self.gripper.go()
		rospy.sleep(1)

	def place_objects(self):
		self.gripper.set_named_target("open_claw")
		rospy.loginfo("start place!")
        	self.gripper.go()
		rospy.sleep(1)
	


class MoveItCartesianDemo:

    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_cartesian_demo', anonymous=True)

        # 是否需要使用笛卡尔空间的运动规划
        cartesian = rospy.get_param('~cartesian', True)

        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('arm_group')

	## 初始化gripper
	pickplace = MoveitPickPlaceDemo()

        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)

        # 设置目标位置所使用的参考坐标系
        arm.set_pose_reference_frame('base_link')

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.01)

        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.05)
        arm.set_max_velocity_scaling_factor(0.05)

        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
	
	# #抓手垂直于地面四元数是: 0.491657,0.508185, 0.50816, -0.491726 
	grasp_home = [-0.0114585329053,0.707019774401, 0.70700930302, 0.0113835878671]
	# 获取当前位姿数据最为机械臂运动的起始位姿
        start_pose = arm.get_current_pose(end_effector_link).pose

	# 设置路点数据，并加入路点列表
        wpose = deepcopy(start_pose)
	#wpose.position.z += 0.2
	arm.set_pose_target(grasp_home)
        arm.go()
        rospy.sleep(1)

	#抓手松开
	#pickplace.place_objects()

	'''print("the first end is :", wpose)
	wpose.orientation.x -= -0.0114585329053- grasp_home[0]
	wpose.orientation.y -= 0.707019774401 - grasp_home[1]
	wpose.orientation.z -= 0.70700930302 - grasp_home[2]
	wpose.orientation.w -= 0.0113835878671 - grasp_home[3]
	print("the second end is :", wpose)
	arm.set_pose_target(wpose)
        arm.go()'''
        rospy.sleep(1)

	
	
	
	




if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass
