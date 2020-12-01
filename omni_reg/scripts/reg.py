#!/usr/bin/env python
# coding=utf8

import rospy
import numpy as np
import tf
import time

from geometry_msgs.msg import PoseStamped, Point, Twist, TwistStamped
from geometry_tools import *

#PID conf
vel_kP = 0.8
vel_kD = 0.1
vel_limit = 0.7

rot_kP = 0.8
rot_kD = 0.1
rot_limit = 0.7


# init values
goal_pose =  None
current_pose = None
current_course = None
goal_course = None

goal_tolerance = 0.3					# tolerance to target point in meter
angle_tolerance = np.deg2rad(10)		# tolerance to target angle in degree

# main time
main_timer = 0.

def CurrentPoseClb(data):
	"""
	Получаем положение дрона
	:param data: PaseStamped дрона
	:return:
	"""
	global current_pose, current_course

	current_pose = point_to_np(data.pose.position)
	current_course = get_yaw_from_quat(data.pose.orientation)

def GoalPoseClb(data):
	"""
	Get goal point
	@param data:
	@return:
	"""
	global goal_pose, goal_course

	goal_pose = point_to_np(data.pose.position)
	goal_course = get_yaw_from_quat(data.pose.orientation)


def point_to_pose_sp(point, yaw=None, frame_id = "map"):
	"""
	Returse pose stamped from point
	"""
	pose_sp_msgs = PoseStamped()
	pose_sp_msgs.header.frame_id = frame_id
	pose_sp_msgs.header.stamp = rospy.Time.now()
	pose_sp_msgs.pose.position.x = point[0]
	pose_sp_msgs.pose.position.y = point[1]
	pose_sp_msgs.pose.position.z = point[2]

	if yaw is None:
		pose_sp_msgs.pose.orientation.w = 1.
	else:
		quat = get_quat_from_yaw(yaw)
		pose_sp_msgs.pose.orientation.x = quat[0]
		pose_sp_msgs.pose.orientation.y = quat[1]
		pose_sp_msgs.pose.orientation.z = quat[2]
		pose_sp_msgs.pose.orientation.w = quat[3]

	return pose_sp_msgs

# Control
def limit(val, lim_min, lim_max):
	if val > lim_max:
		val = lim_max
	elif val < lim_min:
		val = lim_min
	return val


"""
Main loop
"""
if __name__ == '__main__':
	rospy.init_node("omni_reg_node", anonymous=True)
	rate = rospy.Rate(30.0)

	# Subscribers
	rospy.Subscriber("/goal", PoseStamped, GoalPoseClb)
	rospy.Subscriber("/robot/local_position", PoseStamped, CurrentPoseClb)

	cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)


	# Init time
	old_time = time.time()

	cmd_msg = Twist()

	last_error_pose = np.array([0.,0.,0.])
	last_error_course = 0.0

	while not rospy.is_shutdown():
		# Timers update
		if goal_pose is None or current_pose is None or current_course is None or goal_course is None:
			continue

		dt = time.time() - old_time
		old_time = time.time()

		error_pose = goal_pose - current_pose

		error_course = goal_course - current_course
		error_course = normalize_angle(error_course)


		# Publish cmd

		vel_lin_vec = (error_pose - last_error_pose) / dt

		vel_rot_vec = (error_course-last_error_course)/dt



		reg_hor_vec = error_pose*vel_kP+vel_lin_vec*vel_kD
		reg_rot = (error_course*rot_kP)+(vel_rot_vec*rot_kD)

		rot_vec = rotate_point2d(-current_course, reg_hor_vec)

		cmd_msg.linear.x = rot_vec[0]
		cmd_msg.linear.y = rot_vec[1]
		cmd_msg.angular.z = reg_rot


		cmd_pub.publish(cmd_msg)
		print(cmd_msg)

		last_error_pose = error_pose
		last_error_course = error_course


		rate.sleep()