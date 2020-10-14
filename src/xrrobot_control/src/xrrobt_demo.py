#!/usr/bin/env python

# https://github.com/reem-utils/reem_snippets/tree/master/scripts

import time
import roslib
import rospy
import actionlib
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Point, Quaternion, Pose
from moveit_msgs.msg import MoveGroupGoal, MoveGroupResult, MoveGroupAction, Constraints, PositionConstraint, OrientationConstraint, MoveItErrorCodes, RobotState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
import moveit_commander
from math import pi

class MoveMK2ik():
    def __init__(self):

        rospy.init_node("move_mk2_ik")
        rospy.loginfo("Connecting to move_group AC")
        self.moveit_ac = actionlib.SimpleActionClient('/move_group', MoveGroupAction)
        self.moveit_ac.wait_for_server()
        rospy.loginfo("Succesfully connected.")

        group = moveit_commander.MoveGroupCommander("arm_group")
        print( "Current pose is: ")
        print( group.get_current_pose().pose)

        r = rospy.Rate(10)
        self.calculate_mk2_ik()
        
        while not rospy.is_shutdown():
            
            r.sleep()

    def create_move_group_pose_goal(self, goal_pose=Pose(), group="arm_group", end_link_name=None, plan_only=True):

        header = Header()
        header.frame_id = 'base_link'
        header.stamp = rospy.Time.now()

        moveit_goal = MoveGroupGoal()
        goal_c = Constraints()
        position_c = PositionConstraint()
        position_c.header = header
        if end_link_name != None: 
            position_c.link_name = end_link_name
        position_c.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])) 
        position_c.constraint_region.primitive_poses.append(goal_pose)
        position_c.weight = 1.0
        goal_c.position_constraints.append(position_c)
        orientation_c = OrientationConstraint()
        orientation_c.header = header
        if end_link_name != None:
            orientation_c.link_name = end_link_name
        orientation_c.orientation = goal_pose.orientation
        orientation_c.absolute_x_axis_tolerance = 3.14
        orientation_c.absolute_y_axis_tolerance = 3.14
        orientation_c.absolute_z_axis_tolerance = 3.14
        orientation_c.weight = 1.0
        goal_c.orientation_constraints.append(orientation_c)
        moveit_goal.request.goal_constraints.append(goal_c)
        moveit_goal.request.num_planning_attempts = 5 
        moveit_goal.request.allowed_planning_time = 5.0
        moveit_goal.planning_options.plan_only = plan_only
        moveit_goal.planning_options.planning_scene_diff.is_diff = True # Necessary
        moveit_goal.request.group_name = group
    
        return moveit_goal

    def calculate_mk2_ik(self):

        group = moveit_commander.MoveGroupCommander("arm_group")
        goal_pose = Pose()

        # Goto position 1 (Home Position)
        goal_point = Point(0.373429,0.0420103,0.453522)#x ,y , z
        goal_pose.position = goal_point
        #quat = quaternion_from_euler(0, 0, -0.3) # roll, pitch, yaw
        #goal_pose.orientation = Quaternion(*quat.tolist())
        goal_pose.orientation.x = 0.0
        goal_pose.orientation.y = 0.707098
        goal_pose.orientation.z = 0.707116
        goal_pose.orientation.w = 0.0
        moveit_goal = self.create_move_group_pose_goal(goal_pose, group="arm_group", end_link_name="grasping_frame", plan_only=False)
        rospy.loginfo("Sending goal...")
        self.moveit_ac.send_goal(moveit_goal)
        rospy.loginfo("Waiting for result...")
        self.moveit_ac.wait_for_result(rospy.Duration(10.0))
        moveit_result = self.moveit_ac.get_result()
        time.sleep(1)
        print( "xrrobot pose 1 is: ")
        print( group.get_current_pose().pose)

        group.clear_pose_targets()

        # Goto position 2 (Pick Position)
        goal_point = Point(0.373687,0.0410177,0.0933609)#x ,y , z
        goal_pose.position = goal_point
        #quat = quaternion_from_euler(0, 0, -0.3) # roll, pitch, yaw
        #goal_pose.orientation = Quaternion(*quat.tolist())
        goal_pose.orientation.x = 0.0
        goal_pose.orientation.y = 0.707027
        goal_pose.orientation.z = 0.707165
        goal_pose.orientation.w = 0.005
        moveit_goal = self.create_move_group_pose_goal(goal_pose, group="arm_group", end_link_name="grasping_frame", plan_only=False)
        rospy.loginfo("Sending goal...")
        self.moveit_ac.send_goal(moveit_goal)
        rospy.loginfo("Waiting for result...")
        self.moveit_ac.wait_for_result(rospy.Duration(10.0))
        moveit_result = self.moveit_ac.get_result()
        time.sleep(1)
        print( "xrrobot pose 2 is: ")
        print( group.get_current_pose().pose)

        group.clear_pose_targets()

        # Goto position 3 (Home Position)
        goal_point = Point(0.373429,0.0420103,0.453522)#x ,y , z
        goal_pose.position = goal_point
        #quat = quaternion_from_euler(0, 0, -0.3) # roll, pitch, yaw
        #goal_pose.orientation = Quaternion(*quat.tolist())
        goal_pose.orientation.x = 0.0
        goal_pose.orientation.y = 0.707098
        goal_pose.orientation.z = 0.707116
        goal_pose.orientation.w = 0.0
        moveit_goal = self.create_move_group_pose_goal(goal_pose, group="arm_group", end_link_name="grasping_frame", plan_only=False)
        rospy.loginfo("Sending goal...")
        self.moveit_ac.send_goal(moveit_goal)
        rospy.loginfo("Waiting for result...")
        self.moveit_ac.wait_for_result(rospy.Duration(10.0))
        moveit_result = self.moveit_ac.get_result()
        time.sleep(1)
        print( "xrrobot pose 3 is: ")
        print( group.get_current_pose().pose)

        group.clear_pose_targets()

if __name__ == '__main__':
    try:
        s = MoveMK2ik()
        rospy.spin()
    except rospy.ROSInterruptException: pass
