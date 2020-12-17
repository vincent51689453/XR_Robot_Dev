#!/usr/bin/env python


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

class MoveMK2ik():
    def __init__(self):

        rospy.init_node("move_mk2_ik")
        rospy.loginfo("Connecting to move_group AC")
        self.moveit_ac = actionlib.SimpleActionClient('/move_group', MoveGroupAction)
        self.moveit_ac.wait_for_server()
        rospy.loginfo("Succesfully connected.")

        group = moveit_commander.MoveGroupCommander("manipulator")
        print( "Current pose is: ")
        print( group.get_current_pose().pose)

        r = rospy.Rate(10)
	self.calculate_mk2_ik()
        while not rospy.is_shutdown():
            #self.calculate_mk2_ik()
            r.sleep()

    def create_move_group_pose_goal(self, goal_pose=Pose(), group="manipulator", end_link_name=None, plan_only=True):

        header = Header()
        header.frame_id = 'base_link'
        header.stamp = rospy.Time.now()

        moveit_goal = MoveGroupGoal()
        goal_c = Constraints()
        position_c = PositionConstraint()
        position_c.header = header
        if end_link_name != None: 
            position_c.link_name = end_link_name
        position_c.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.1])) 
        position_c.constraint_region.primitive_poses.append(goal_pose)
        position_c.weight = 1.0
        goal_c.position_constraints.append(position_c)
        orientation_c = OrientationConstraint()
        orientation_c.header = header
        if end_link_name != None:
            orientation_c.link_name = end_link_name
        orientation_c.orientation = goal_pose.orientation
        orientation_c.absolute_x_axis_tolerance = 0.01
        orientation_c.absolute_y_axis_tolerance = 0.01
        orientation_c.absolute_z_axis_tolerance = 0.01
        orientation_c.weight = 0.01
        goal_c.orientation_constraints.append(orientation_c)
        moveit_goal.request.goal_constraints.append(goal_c)
        moveit_goal.request.num_planning_attempts = 5 
        moveit_goal.request.allowed_planning_time = 5.0
        moveit_goal.planning_options.plan_only = plan_only
        moveit_goal.planning_options.planning_scene_diff.is_diff = True # Necessary
        moveit_goal.request.group_name = group
    
        return moveit_goal

    def calculate_mk2_ik(self):

        #goal_point = Point(0.298, -0.249, -0.890) home position
        group = moveit_commander.MoveGroupCommander("manipulator")
        goal_pose = Pose()

        # Goto position 1
        # x ,y , z
        goal_point = Point(-0.0297,0.3491,0.4051)
        goal_pose.position = goal_point
        # roll, pitch, yaw
        quat = quaternion_from_euler(0.0, 0.0, 0.0) 
        # hardcode quaternion
        goal_pose.orientation = Quaternion(0,0,0.707,0.707)
        moveit_goal = self.create_move_group_pose_goal(goal_pose, group="manipulator", end_link_name="end_Link", plan_only=False)
        rospy.loginfo("Sending goal...")
        self.moveit_ac.send_goal(moveit_goal)
        rospy.loginfo("Waiting for result...")
        self.moveit_ac.wait_for_result(rospy.Duration(10.0))
        moveit_result = self.moveit_ac.get_result()
        time.sleep(1)
        print( "Home position: ")
        print( group.get_current_pose().pose)
	

        # Goto position 2
        goal_point = Point(-0.287542,0.319282,0.134797)
        goal_pose.position = goal_point
        quat = quaternion_from_euler(0.2, 0.2, 0.2) # roll, pitch, yaw
        goal_pose.orientation = Quaternion(00.011906,-0.0238,0.904829 ,0.424961)
        moveit_goal = self.create_move_group_pose_goal(goal_pose, group="manipulator", end_link_name="end_Link", plan_only=False)
        rospy.loginfo("Sending goal...")
        self.moveit_ac.send_goal(moveit_goal)
        rospy.loginfo("Waiting for result...")
        self.moveit_ac.wait_for_result(rospy.Duration(10.0))
        moveit_result = self.moveit_ac.get_result()
        time.sleep(1)
        print "Pick position: "
        print group.get_current_pose().pose


if __name__ == '__main__':
    try:
        s = MoveMK2ik()
        rospy.spin()
    except rospy.ROSInterruptException: pass
