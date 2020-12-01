#!/usr/bin/env python
# coding: utf-8

"""
The node for publish odometry to PX4(mavros/pixhawk)
"""

import rospy
from rospy import Header
from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

pose_msg = PoseStamped()
odom_msg = Odometry()

roll_offset = pitch_offset = yaw_offset = 0.

def q_mult(q1, q2):
	x1, y1, z1, w1 = q1
	x2, y2, z2, w2 = q2
	w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
	x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
	y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
	z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
	return np.array([x, y, z, w])

def odometry_clb(data):
	"""
	Odometry callback.
	:param data: NavSatFix
	:return:
	"""
	global odom_pub, roll_offset, pitch_offset, yaw_offset, odom_msg, map_link

	odom_msg = data
	odom_msg.header.frame_id = map_link

	# Rotate axis
	if roll_offset != 0. or pitch_offset != 0 or yaw_offset != 0.:
		q = data.pose.pose.orientation
		q = np.array([q.x, q.y, q.z, q.w])
		rotation_quaternion = quaternion_from_euler(roll_offset, pitch_offset, yaw_offset)
		result = q_mult(q, rotation_quaternion)

		odom_msg.pose.pose.orientation.x = result[0]
		odom_msg.pose.pose.orientation.y = result[1]
		odom_msg.pose.pose.orientation.z = result[2]
		odom_msg.pose.pose.orientation.w = result[3]

	# Publish
	odom_pub.publish(odom_msg)

def cmd_vel_clb(data):
	global cmd_vel_pub
	cmd_vel_pub.publish(data.twist)

def getQuatToEuler(x, y, z, w):
	"""
	Transform quaternion to euler angels
	:param x:
	:param y:
	:param z:
	:param w:
	:return: euler angels
	"""
	# type(pose) = geometry_msgs.msg.Pose
	euler = tf.transformations.euler_from_quaternion((x,y,z,w))
	roll = euler[0]
	pitch = euler[1]
	yaw = euler[2]
	return roll,pitch,yaw

def getEulerToQuat(roll=0., pitch=0., yaw = 0.):
	"""
	Transform euler angels to quaternion
	:param roll:
	:param pitch:
	:param yaw:
	:return: quaternion
	"""
	# type(pose) = geometry_msgs.msg.Pose
	q = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
	quat = Quaternion()
	quat.x = q[0]
	quat.y = q[1]
	quat.z = q[2]
	quat.w = q[3]
	return quat

# rotate vector v1 by quaternion q1
def qv_mult(quat, v1):
	q1 = [quat.x, quat.y, quat.z, quat.w]
	v1 = tf.transformations.unit_vector(v1)
	q2 = list(v1)
	q2.append(0.0)
	return tf.transformations.quaternion_multiply(
		tf.transformations.quaternion_multiply(q1, q2),
		tf.transformations.quaternion_conjugate(q1)
	)[:3]

if __name__ == '__main__':

	rospy.init_node('tf_to_ps_node', anonymous=True)
	rate = rospy.Rate(30.)

	listener = tf.TransformListener()


	base_link = "base_link"
	map_link = "odom"

	base_link = rospy.get_param("~base_link", base_link)
	map_link = rospy.get_param("~map_link", map_link)

	roll_offset = rospy.get_param("~roll_offset", roll_offset)
	pitch_offset = rospy.get_param("~pitch_offset", pitch_offset)
	yaw_offset = rospy.get_param("~yaw_offset", yaw_offset)


	# subscriber
	rospy.Subscriber("/t265/odom/sample", Odometry, odometry_clb)
	rospy.Subscriber("/robot/cmd_vel_stamp", TwistStamped, cmd_vel_clb)

	cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

	pose_pub = rospy.Publisher("/robot/local_position", PoseStamped, queue_size=10)
	odom_pub = rospy.Publisher("/robot/odom", Odometry, queue_size=10)

	pose_msg.header.frame_id = map_link

	while rospy.is_shutdown() is False:
		try:
			(trans,rot) = listener.lookupTransform(map_link, base_link, rospy.Time(0))
			pose_msg.header.stamp = rospy.Time.now()
			pose_msg.pose.position.x = trans[0]
			pose_msg.pose.position.y = trans[1]
			pose_msg.pose.position.z = trans[2]

			pose_msg.pose.orientation.x = rot[0]
			pose_msg.pose.orientation.y = rot[1]
			pose_msg.pose.orientation.z = rot[2]
			pose_msg.pose.orientation.w = rot[3]

			pose_pub.publish(pose_msg)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		rate.sleep()

