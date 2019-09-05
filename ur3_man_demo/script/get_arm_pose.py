#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import signal
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf.transformations

from std_msgs.msg import String

running = False

def signal_handler(signum, frame):
	global running
	running = False

if __name__ == '__main__':
	signal.signal(signal.SIGINT, signal_handler)
	signal.signal(signal.SIGTERM, signal_handler)

	rospy.init_node("get_ur_pose")

	moveit_commander.roscpp_initialize(sys.argv)
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	group = moveit_commander.MoveGroupCommander("left_arm")
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=5)
	rospy.sleep(3)

	running = True
	while running:
		tcp_pose = group.get_current_pose()
		joint_list = group.get_current_joint_values()
		print('===============')
		print(tcp_pose.pose)
		print(joint_list)
		rospy.sleep(1)
