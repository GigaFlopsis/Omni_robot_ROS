#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Node for connect to arduino and publish some daate
"""

import rospy
import math
from geometry_msgs.msg import Twist
import serial
import numpy

serial_init = False
# Global variables

port = '/dev/ttyACM0'
baudrate = 1000000
msg = ""
last_msg = ""

def toFixed(numObj, digits=0):
	return float('{:.3f}'.format(numObj))

def callbackCmd(data):
	"""
	Get data from cmd_vel
	:param data:
	:return:
	"""
	global msg, last_msg
	# if serial_init is False:
	# 	return
	scale = 5.
	msg = "v %s %s %s" %(toFixed(data.linear.x*scale, 2),
						   toFixed(data.angular.z*scale, 2),
						   toFixed(data.angular.x*scale*0.5, 2))

	if last_msg != msg:
		# print(msg)
		SendToSerial(bytearray(msg))

		last_msg = last_msg;

def SendToSerial(data):
	global ser
	try:
		ser.write(data)
	except:
		print("Error. Serial is closed")
		ser.close()
		ser.open()
if __name__ == '__main__':
	# inti node
	rospy.init_node('arduino_serial_node', anonymous=True)
	rate = rospy.Rate(5.0)

	# Get params
	port = rospy.get_param("port", port)
	baudrate = rospy.get_param("baudrate", baudrate)


	# Init serial
	ser  = serial.Serial(port, baudrate, timeout=0, parity=serial.PARITY_EVEN, rtscts=1)

	if ser.is_open is False:
		print("Cant open port, exit")
		exit()
	line = []
	rospy.sleep(2.)

	rospy.Subscriber("/cmd_vel", Twist, callbackCmd)
	rospy.spin()
	while rospy.is_shutdown() is False:

		# for i in numpy.arange(0, 1, 0.02):
		# 	msg = "v %s %s %s" %(toFixed(i, 2), 0.0, 0.0)

		# print("ok")
		# for k in numpy.arange(1, 0, -0.02):
		# 	msg = "v %s %s %s" %(toFixed(k, 2), 0.0, 0.0)
		# 	print(msg)
		# 	SendToSerial(msg)
		# reading = ser.readline().decode()
		# print(reading)
		rate.sleep()

	print("Exit")
	ser.close()