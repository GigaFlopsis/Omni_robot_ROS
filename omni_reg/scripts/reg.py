#!/usr/bin/env python
# coding=utf8

# Global planner for task 1
# 1.  take off
# 2.1 run exploration
# 2.2 solve boxes postion
# 3. go to home
# 4. landing

import rospy
import numpy as np
import tf
import time
import thread

from geometry_msgs.msg import PoseStamped, Point

from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Bool, Empty

from mavros_msgs.srv import SetMode, CommandBool

from sklearn.cluster import DBSCAN

from nbv_msgs.msg import State, Status
from nbv_msgs.srv import SetState, SetStateRequest
from debug_tools import *
from modes import *
from geometry_tools import *

# init values

fly_mode = Fly_modes.NOT_INIT
mission_state = Mission_state.WAIT
exploration_state = State()


goal_pose =  []
current_pose = []
init_pose = []
init_course = []
current_course = []
goal_course = []

landing_planner_state = False
landing_pose = []
landing_course = []


mission_list = [[4.0, 0, 1, 0],
				[4.0, 3.5 , 2.3, 0],
				[14, 3.5, 2.3, 0],
				[14, -5.5 , 2.3, 0],
				[8, -5., 2.3, 0],
				[8, 0., 2.3, 0]
				]
mission_revers = [[1, 0 , 2, 0],
				  [0, 0 , 3, 0]
				  ]

use_revers = True
use_autoarm = False

goal_tolerance = 2.					# tolerance to target point in meter
angle_tolerance = np.deg2rad(360)		# tolerance to target angle in degree

skip_go_home = False
debug_mode = True

#box params
find_objects_array = np.array([])
boxes_points = []
box_max_count = 5					# maximum count of found box
box_angle_lim = np.deg2rad(20)		# +- max degree for count box
box_z_offset = -0.3
box_timer = 0.
box_rate = 0.5			# hz rof solve boxes position


# main time
main_timer = 0.
main_max_time = 60.*20		# max seconds

# checking flags
check_exploration = True
check_landing = True


def init_check():
	"""
	Cheking init params for starm global planner
	@return:
	"""
	global fly_mode, check_exploration
	if	len(current_pose) == 0 or \
			len(current_course) == 0 or \
			check_exploration is False or \
			check_landing is False:
		fly_mode = Fly_modes.NOT_INIT
	else:
		print("Init params!")
		fly_mode = Fly_modes.INIT_STATE

def LandingDetectorClb(data):
	"""
	Get landing plane position
	"""
	global landing_pose, landing_course

	landing_pose = point_to_np(data.pose.position)
	landing_course = [get_yaw_from_quat(data.pose.orientation)]
	print("Get landing position")

def BoxDetectorClb(data):
	"""
	Get box detector position
	if  -0.5<x<0.5 add to np list of position
	:param data:
	:return:
	"""
	global find_objects_array

	# if data.pose.position.z  < -0.5 or data.pose.position.z > 0.5:
	# 	# print("ERROR! find object: bag height", data.pose.position.z)
	# 	return
	# roll, pitch, yaw = euler_from_quaternion(data.pose.orientation)
	# if abs(roll) > box_angle_lim or abs(pitch) > box_angle_lim:
	# 	# print("ERROR!	bag angles of box: \n\troll: %f\n\tpitch: %f" % (np.rad2deg(roll),np.rad2deg(pitch)))
	# 	return

	find_poin = point_to_np(data.pose.position)
	find_poin[2] = find_poin[2] + box_z_offset
	if len(find_objects_array) == 0:
		find_objects_array = [find_poin]

	find_objects_array = np.concatenate((find_objects_array, [find_poin]), axis=0)

def Cluster_points(array):
	pred = DBSCAN(eps=2.0, min_samples=2).fit(array)
	labels = pred.labels_

	n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
	obs_centers = []

	for cluster_index in range(0, n_clusters_):
		indices = np.where(labels == cluster_index)
		if(len(indices)>0):
			custer = array[indices]
			cluster_mean = np.median(custer, axis=0)
			obs_centers.append(cluster_mean)

	return np.array(obs_centers)

def CurrentPoseClb(data):
	"""
	Получаем положение дрона
	:param data: PaseStamped дрона
	:return:
	"""
	global init_pose, current_pose, current_course, init_course

	# set init position
	if len(current_pose) == 0:
		init_pose = point_to_np(data.pose.position)
		init_course = [get_yaw_from_quat(data.pose.orientation)]
		print("set init position", init_pose)

	current_pose = point_to_np(data.pose.position)
	current_course = [get_yaw_from_quat(data.pose.orientation)]

def GoalPoseClb(data):
	"""
	Get goal point
	@param data:
	@return:
	"""
	global goal_pose, goal_course

	goal_pose = point_to_np(data.pose.position)
	goal_course = [get_yaw_from_quat(data.pose.orientation)]

def Auto_arm():
	"arming ang change to OFFBOARD"

	try:
		set_mode_ = rospy.ServiceProxy("mavros/set_mode", SetMode)
		set_mode_(custom_mode="OFFBOARD")
		time.sleep(1)
		set_arm_ = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
		set_arm_(value=True)
		print("Arm successfully!")
	except:
		print("ERROR ARMING")

def start_clb_srv(req):
	"""
	Service when start of state planner

	:param req:
	:return:
	"""
	global mission_state, fly_mode, mission_current_item
	resp = TriggerResponse()

	if mission_state == Mission_state.WAIT and fly_mode != Fly_modes.NOT_INIT:
		resp.success = True
		resp.message = "Start global planner: True"
		mission_state = Mission_state.RUN
		fly_mode = Fly_modes.TAKE_OFF
		if use_autoarm:
			thread.start_new_thread(Auto_arm, ())
		mission_current_item = 0
	else:
		resp.success = False
		resp.message = "Start mission ERROR: the firs reset mission"
	return resp



def explorationClb(data):
	"Get current state of explotation"
	global exploration_state, check_exploration
	check_exploration = True
	exploration_state = data

def reset_clb_srv(req):
	"""
	Service when reset of state planner

	:param req:
	:return:
	"""
	global mission_state, planner_mode, goal_sub, fly_mode

	print("set reset:")

	mission_state = Mission_mode.WAIT
	# goal_sub = rospy.Subscriber("/goal_pose", Goal, goal_clb)
	resp = TriggerResponse()
	resp.success = True
	resp.message = "Reset state: True"

	mission_state = Mission_mode.WAIT
	if fly_mode != Fly_modes.NOT_INIT:
		fly_mode = Fly_modes.NOT_INIT

	thread.start_new_thread(exploration_reset,())

	return resp

def go_home_clb_srv(req):
	"""
	Service for go state planner

	:param req:
	:return:
	"""
	global mission_state, planner_mode, goal_sub, fly_mode, use_revers, mission_current_item

	print("go home:")

	resp = TriggerResponse()
	resp.success = True
	resp.message = "Reset state: True"

	exploratiom_go_home()

	if fly_mode != Fly_modes.NOT_INIT:
		fly_mode = Fly_modes.GO_HOME
	if use_revers:
		fly_mode = Fly_modes.REVERSE_MISSION
	return resp

"""
Exploration srv
"""
def exploration_start():
	if exploration_state.status.status != Status.STATUS_WAIT:
		print("is running")
		return
	print("CALL START")
	thread.start_new_thread(callStartExploration, ())


def callStartExploration():
	try:
		rospy.wait_for_service("nbv_planner/set_mode")
		set_state_ = rospy.ServiceProxy("nbv_planner/set_mode", SetState)
		req = SetStateRequest()
		req.status.status = Status.STATUS_RUN
		set_state_(req)
		print("Send Exploration: RUN")
	except:
		print("ERRoR start exploration")

def exploration_reset():
	try:
		set_reset_state_ = rospy.ServiceProxy("/nbv_planner/set_mode", SetState)
		req = SetStateRequest()
		req.status.status = Status.STATUS_RESET
		set_reset_state_(req)
		print("Send Exploration: STOP")
		set_arm_ = rospy.ServiceProxy("drone_landing/reset", Trigger)
		set_arm_()
	except:
		print("ERROR reset exploration")

def LandingPlannerState(data):
	global 	landing_planner_state, check_landing
	check_landing = True

	landing_planner_state = data


def exploratiom_go_home():
	global landing_pose, go_home_pub, mission_current_item, fly_mode
	if exploration_state.status.status == Status.STATUS_GO_HOME or exploration_state.status.status == Status.STATUS_GO_HOME_COMPLETE:
		print("is running GO HOME")
		return

	if use_revers:
		fly_mode = Fly_modes.REVERSE_MISSION
		mission_current_item = 0
		thread.start_new_thread(stop_planner, ())
	else:
		fly_mode = Fly_modes.GO_HOME
		thread.start_new_thread(call_go_home, ())

	landing_pose = []


def call_go_home():
	# go_home_pub.publish()

	try:
		rospy.wait_for_service("nbv_planner/set_mode")
		set_state_ = rospy.ServiceProxy("/nbv_planner/set_mode", SetState)
		req = SetStateRequest()
		req.status.status = Status.STATUS_GO_HOME
		set_state_(req)
		print("sent go home is complited")
	except:
		print("GO HOME ERROR")

def stop_planner():
	# go_home_pub.publish()

	try:
		rospy.wait_for_service("nbv_planner/set_mode")
		set_state_ = rospy.ServiceProxy("/nbv_planner/set_mode", SetState)
		req = SetStateRequest()
		req.status.status = Status.STATUS_STOP
		set_state_(req)
		print("sent stop planner")
	except:
		print("STOP PLANNER ERROR")

def switch_landing():
	"""
	Switch to landing mode
	@return:
	"""
	global fly_mode
	exploration_reset()
	fly_mode = Fly_modes.LAND
	try:
		# Call landing
		set_arm_ = rospy.ServiceProxy("drone_landing/start", Trigger)
		set_arm_()
	except:
		print("ERROR call drone landing node")

"""
Mission functions
"""

def move_to_point(_goal_pose, _goal_course, _curren_pose, _current_cource, _point_tolerance, _angle_tolerance, use2d=False):
	"""
	Move to target point
	if hit to point, return true, else  false
	"""
	if use2d:
		dist_to_point = np.linalg.norm([_curren_pose[0]-_goal_pose[0],_curren_pose[1]-_goal_pose[1]])
	else:
		dist_to_point = np.linalg.norm(_curren_pose-_goal_pose)
	course_error = abs(normalize_angle(_current_cource) - normalize_angle(_goal_course))
	print("error to point", dist_to_point, course_error)

	if dist_to_point < _point_tolerance and course_error < _angle_tolerance:
		return True
	else:
		return False

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


"""
Main loop
"""
if __name__ == '__main__':
	rospy.init_node("drone_global_planner_node", anonymous=True)
	rate = rospy.Rate(5.0)

	# Subscribers
	rospy.Subscriber("/goal", PoseStamped, GoalPoseClb)
	rospy.Subscriber("/mavros/local_position/pose", PoseStamped, CurrentPoseClb)
	rospy.Subscriber("nbv_planner/state", State, explorationClb)
	rospy.Subscriber("/find_object", PoseStamped, BoxDetectorClb)
	rospy.Subscriber("/landing_place", PoseStamped, LandingDetectorClb)
	rospy.Subscriber("/landing_planner/state", Bool, LandingPlannerState)


	# # Publishers
	# activePub = rospy.Publisher(activeStatusTopic, Bool, queue_size=10)
	# statusPub = rospy.Publisher(setGoalStatusTopic, Bool, queue_size=10)
	go_home_pub = rospy.Publisher("nbv_planner/go_home", Empty, queue_size=10)
	goalPub = rospy.Publisher("/goal", PoseStamped, queue_size=10)
	box_coords_pub = rospy.Publisher("/object_coordinates", Point,queue_size=10)

	if debug_mode:
		box_markers = MarkerDebug(_topic="/global_planner/markers/boxes")

	"""
	Service
	"""
	start_srv = rospy.Service("global_planner/start", Trigger,start_clb_srv)
	reset_srv = rospy.Service("global_planner/reset", Trigger,reset_clb_srv)
	go_home_src = rospy.Service("global_planner/go_home", Trigger,go_home_clb_srv)

	# Init time
	old_time = time.time()
	box_current_count = 0

	mission_current_item = 0

	while not rospy.is_shutdown():
		# Timers update
		dt = time.time() - old_time
		old_time = time.time()

		box_timer += dt
		main_timer += dt


		# Waint command from operator
		if mission_state == Mission_state.WAIT:
			main_timer = 0.
			start_clb_srv(None)

		if main_timer < main_max_time:
			print("time: %f/%f" % (main_timer, main_max_time))

		if mission_state == Mission_state.RUN and main_timer > main_max_time:
			"""
			Go to home
			"""
			if fly_mode == Fly_modes.EXPLORATION:
				print("Mission time timeout: GO TO HOME!")
				fly_mode = Fly_modes.GO_HOME
				exploratiom_go_home()
				continue


		if fly_mode == Fly_modes.NOT_INIT:
			print("Global planner not run! Wait inti")
			init_check()


		elif fly_mode == Fly_modes.INIT_STATE:
			print("INIT STATE! Wain command from operator.")
			rospy.sleep(1.)

		elif fly_mode == Fly_modes.TAKE_OFF:
			"""
			Take off and to by mission
			"""
			print("Fly_modes.TAKE_OFF: mission item: %d/%d" % (mission_current_item+1, len(mission_list)))
			### fly by mission list
			if (len(mission_list) == 0):
				print("mission is empty. Switch mode to EXPLORATION")
				exploration_start()
				fly_mode = Fly_modes.EXPLORATION
				continue

			if mission_current_item >= len(mission_list):
				print("mission complite.  Switch mode to EXPLORATION")
				time.sleep(2.0)
				fly_mode = Fly_modes.EXPLORATION
				exploration_start()
				continue

			target_point = mission_list[mission_current_item]
			print("target_point:", target_point)
			goal_state = move_to_point(_curren_pose=current_pose, _current_cource=current_course[0],
						  _goal_pose=[target_point[0], target_point[1], target_point[2]],_goal_course=np.deg2rad(target_point[3]),
						  _point_tolerance=goal_tolerance, _angle_tolerance=angle_tolerance)

			goalPub.publish(point_to_pose_sp([target_point[0], target_point[1], target_point[2]], np.deg2rad(target_point[3])))

			if goal_state:
				mission_current_item += 1
				time.sleep(3.0)

			## Solve boxes position
			if (box_timer > 1./box_rate):
				box_timer = 0.
				if len(find_objects_array) > 0:
					boxes_points = Cluster_points(find_objects_array)

				if (box_current_count != len(boxes_points)):
					box_current_count = len(boxes_points)
					print("box_current_count:", box_current_count)
					box_markers.drawBoxes(boxes_points)
					print(boxes_points)
					# Publish point
					point_msgs = Point()
					box =  boxes_points[box_current_count-1]
					point_msgs.x = box[0]
					point_msgs.y = box[1]
					point_msgs.z = box[2]

					box_coords_pub.publish(point_msgs)

				if box_current_count == box_max_count:
					print("FOUND ALL BOXES!!!")
					print("GO TO HOME!")
					fly_mode = Fly_modes.GO_HOME
					exploratiom_go_home()
					continue


		elif fly_mode == Fly_modes.EXPLORATION:
			"""
			Exploration mode
			"""
			print("exploration_state:", exploration_state)
			### check exploration
			if exploration_state.status.status == Status.STATUS_COMPLETE:
				fly_mode = Fly_modes.GO_HOME
				exploratiom_go_home()
				continue

			## Solve boxes position
			if (box_timer > 1./box_rate):
				box_timer = 0.
				if len(find_objects_array) > 0:
					boxes_points = Cluster_points(find_objects_array)

				if (box_current_count != len(boxes_points)):
					box_current_count = len(boxes_points)
					print("box_current_count:", box_current_count)
					box_markers.drawBoxes(boxes_points)
					print(boxes_points)
					# Publish point
					point_msgs = Point()
					box =  boxes_points[box_current_count-1]
					point_msgs.x = box[0]
					point_msgs.y = box[1]
					point_msgs.z = box[2]

					box_coords_pub.publish(point_msgs)

					if box_current_count == box_max_count:
						print("FOUND ALL BOXES!!!")
						print("GO TO HOME!")
						fly_mode = Fly_modes.GO_HOME
						exploratiom_go_home()
						continue

		elif fly_mode == Fly_modes.GO_HOME:
			"""
			check if exploration complete or drone in init_pose switch to land mode
			"""
			print("Exploration go home", exploration_state.status.status, fly_mode)

			# if exploration_state.status.status == Status.STATUS_GO_HOME:
			# 	rospy.sleep(0.5)
			if exploration_state.status.status == Status.STATUS_GO_HOME_COMPLETE:
				print("switch to reverse mission")
				fly_mode = Fly_modes.REVERSE_MISSION
				if use_revers:
					mission_current_item = 0
				else:
					mission_current_item = len(mission_list)-1
				continue

			if len(landing_pose) > 0:
				print("see landing plane. Start landing")
				"""
				publish goal pose relative landing plane
				run landing service
				"""
				switch_landing()

		elif fly_mode == Fly_modes.REVERSE_MISSION and use_revers is False:
			"""
			Fly be missiion in reverse state
			"""
			print("fly_mode",fly_mode)

			print("Fly_modes.REVERSE_MISSION: mission item: %d/%d" % (mission_current_item + 1, 0))
			### fly by mission list in reverse
			if (len(mission_list) == 0):
				print("mission is empty. Switch mode to EXPLORATION")
				switch_landing()
				fly_mode = Fly_modes.LAND
				continue

			if mission_current_item < 0:
				print("reverse mission complite.  Switch mode to LAND")
				## publish init pose
				goalPub.publish(point_to_pose_sp([init_pose[0], init_pose[1], 3.], init_course[0]))
				switch_landing()
				fly_mode = Fly_modes.LAND
				continue

			target_point = mission_list[mission_current_item]
			print("target_point:", target_point)
			goal_state = move_to_point(_curren_pose=current_pose, _current_cource=current_course[0],
									   _goal_pose=[target_point[0], target_point[1], target_point[2]],
									   _goal_course=np.deg2rad(target_point[3]),
									   _point_tolerance=goal_tolerance, _angle_tolerance=angle_tolerance)

			goalPub.publish(
				point_to_pose_sp([target_point[0], target_point[1], target_point[2]], np.deg2rad(target_point[3])))

			if goal_state:
				mission_current_item -= 1
				time.sleep(1.0)

			if len(landing_pose) > 0:
				print("see landing plane. Start landing")
				"""
				publish goal pose relative landing plane
				run landing service
				"""
				switch_landing()


			## Solve boxes position
			if (box_timer > 1./box_rate):
				box_timer = 0.
				if len(find_objects_array) > 0:
					boxes_points = Cluster_points(find_objects_array)

				if (box_current_count != len(boxes_points)):
					box_current_count = len(boxes_points)
					print("box_current_count:", box_current_count)
					box_markers.drawBoxes(boxes_points)
					print(boxes_points)
					# Publish point
					point_msgs = Point()
					box =  boxes_points[box_current_count-1]
					point_msgs.x = box[0]
					point_msgs.y = box[1]
					point_msgs.z = box[2]

					box_coords_pub.publish(point_msgs)
					if box_current_count == box_max_count:
						continue


		elif fly_mode == Fly_modes.REVERSE_MISSION and use_revers is True:
			"""
			Fly be missiion in reverse state
			"""
			print("fly_mode",fly_mode)

			if (len(mission_revers) == 0):
				print("mission reset is empty. Switch mode to LAND")
				fly_mode = Fly_modes.LAND
				continue
			print("Fly_modes.REVERSE_MISSION: mission item: %d/%d" % (mission_current_item + 1, 0))

			### fly by mission list in reverse
			if (len(mission_revers) == 0):
				print("mission is empty. Switch mode to EXPLORATION")
				switch_landing()
				fly_mode = Fly_modes.LAND
				continue

			if mission_current_item >= len(mission_revers):
				print("mission complite.  Switch mode to EXPLORATION")
				time.sleep(2.0)
				goalPub.publish(point_to_pose_sp([init_pose[0], init_pose[1], 3.], init_course[0]))
				switch_landing()
				fly_mode = Fly_modes.LAND
				continue

			target_point = mission_revers[mission_current_item]
			print("target_point:", target_point)


			goal_state = move_to_point(_curren_pose=current_pose, _current_cource=current_course[0],
									   _goal_pose=[target_point[0], target_point[1], target_point[2]],
									   _goal_course=np.deg2rad(target_point[3]),
									   _point_tolerance=goal_tolerance, _angle_tolerance=angle_tolerance)

			goalPub.publish(
				point_to_pose_sp([target_point[0], target_point[1], target_point[2]], np.deg2rad(target_point[3])))

			if goal_state:
				mission_current_item += 1
				time.sleep(1.0)

			if len(landing_pose) > 0:
				print("see landing plane. Start landing")
				"""
				publish goal pose relative landing plane
				run landing service
				"""
				switch_landing()

			## Solve boxes position
			if (box_timer > 1./box_rate):
				box_timer = 0.
				if len(find_objects_array) > 0:
					boxes_points = Cluster_points(find_objects_array)

				if (box_current_count != len(boxes_points)):
					box_current_count = len(boxes_points)
					print("box_current_count:", box_current_count)
					box_markers.drawBoxes(boxes_points)
					print(boxes_points)
					# Publish point
					point_msgs = Point()
					box =  boxes_points[box_current_count-1]
					point_msgs.x = box[0]
					point_msgs.y = box[1]
					point_msgs.z = box[2]

					box_coords_pub.publish(point_msgs)
					if box_current_count == box_max_count:
						continue
		elif fly_mode == Fly_modes.LAND:
			"""
			send landing or land on init point
			"""
			"""
			Check status of land
			"""
			if landing_planner_state:
				"just wait"
			else:
				if len(landing_pose) > 0:
					goalPub.publish(point_to_pose_sp([landing_pose[0], landing_pose[1], 0], init_course[0]))
				else:
					goalPub.publish(point_to_pose_sp([init_pose[0], init_pose[1], 0], init_course[0]))

		rate.sleep()