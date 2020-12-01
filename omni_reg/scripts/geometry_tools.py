#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Point
import tf
import tf.transformations

# ---------------------------------------------------------------------------------------------------------------------
# ---------- Геометрические функции

def angle_between(vec1, vec2):
    """
      Функция возвращает угол между двумя двухмерными векторами.

    :param vec1:
    :param vec2:
    :return:
    """

    x = (vec2[1] - vec1[1])  # y
    y = -(vec2[0] - vec1[0])  # -x
    res = math.atan2(x, y) + math.pi

    if res > math.pi:
        res -= 2 * math.pi
    if res < -math.pi:
        res += 2 * math.pi

    return res


def get_dist_to_wall(max_height):
    """
    получаем оптимальное расстояние, до стены, чтобы полностью охватывать её по вертикали

    :param max_height:
    :return:
    """

    angle = math.radians(32.63/2)

    dist_to_wall = 0.5 * (max_height +1)/ math.tan(angle)
    return dist_to_wall + 1


def rotate_vect(rot,dist):
    """
    Поворачиваем на заданный угол относительно a на угол rot
    :param rot: угол порота
    :return: возвращаем точку повёрнутую на нужный угол

    """
    rotate = np.array([[np.cos(rot), -np.sin(rot)],
                       [np.sin(rot), np.cos(rot)]])

    pos = np.array([[dist],[0.0]])
    val = np.dot(rotate,pos)

    return (val[0][0], val[1][0], 0.0)

def rotate_point2d(rot,xy_point):
    """
    Поворачиваем на заданный угол относительно a на угол rot
    :param rot: угол порота
    :return: возвращаем точку повёрнутую на нужный угол

    """
    rotate = np.array([[np.cos(rot), -np.sin(rot)],
                       [np.sin(rot), np.cos(rot)]])
    pos = np.array([[xy_point[0]], [xy_point[1]]])
    val = np.dot(rotate,pos)

    return np.array([val[0][0], val[1][0], 0.0])


def normalize_angle(ang):
    """
    Нормализуем угол от -pi до pi
    :param ang:
    :return:
    """
    if ang > math.pi:
        ang -= 2.0 * math.pi
    elif ang < -math.pi:
        ang += 2.0 * math.pi
    return ang

def deg_course_angle(angle):
    """
    Получаем угол от 0 до 2.pi
    :param angle:
    :return:
    """
    return angle if angle > 0 and angle < math.pi * 2 else math.pi *2 + angle

def rotate_around(pose, speed=0.5):
    """
    Крутимся вокруг своей оси
    :param pose:
    :param speed:
    :return:
    """
    new_angle = pose[3] + (math.radians(1) * speed)
    pose[3] = new_angle % (math.pi *2)

def local_to_global(l_point, current_pos):
    """
    :type l_point: list
    :type current_pos: Pose
    :param l_point:
    :param current_pos:
    :return:
    """
    # print ('orient = %s' % math.degrees(current_pos.orientation.z))
    pose_x = l_point[0]
    pose_y = l_point[1]
    yaw = current_pos.orientation.z
    x_global = pose_x * math.cos(yaw) - pose_y * math.sin(yaw)
    y_global = pose_x * math.sin(yaw) + pose_y * math.cos(yaw)
    z_global = l_point[2] + current_pos.position.z
    return [x_global + current_pos.position.x, y_global + current_pos.position.y, z_global]

def check_line_intersection(walls, found_point):
    """
    :type current_pos: Pose
    :param walls:
    :param current_pos:
    :param found_point:
    :return:
    """


    wall_point = [[walls[0], walls[3]], # низ правый угол
                  [walls[0], walls[1]], # низ левый угол
                  [walls[2], walls[1]], # верх левый угол
                  [walls[2], walls[3]]] # верх правый угол

    dist_to_point = []
    for i in range(len(walls)):
        if i == len(walls)-1:
            dist_to_point.append(dist_to_wall(found_point, wall_point[i], wall_point[0]))
            print ("walls %s: %s" %(i, dist_to_wall(found_point, wall_point[i], wall_point[0])))
        else:
            dist_to_point.append(dist_to_wall(found_point, wall_point[i], wall_point[i+1]))
            print ("walls %s: %s" %(i, dist_to_wall(found_point, wall_point[i], wall_point[i+1])))
    index_wall = dist_to_point.index(min(dist_to_point))

    print ("index wall: %s" % index_wall)
    return index_wall

def dist_to_wall(point, wall_point1, wall_point2):
    """

    :param point:
    :param wall_point1:
    :param wall_point2:
    :return:
    """
    a = (wall_point2[0]-wall_point1[0])*(point[1]-wall_point1[1])-(wall_point2[1]-wall_point1[1])*(point[0]-wall_point1[0])
    b = math.sqrt((wall_point2[0]-wall_point1[0])**2+(wall_point2[1]-wall_point1[1])**2)
    return abs(a/b)

def normalToPath_goal(start_path, end_path, point):
    """
    Метод расчёта точки нормали к траектории

    :param start_path: начальные координаты пути
    :type start_path: Point
    :param end_path: конечные координаты пути
    :type end_path: Point
    :param point:   точка в пространстве
    :type point: Point
    :return: нормализованная точка на траектории
    """
    x1,y1,z1 = start_path.x, start_path.y, start_path.z
    x2, y2, z2 = end_path.x, end_path.y, end_path.z
    x3,y3, z3 = point.x, point.y, point.z

    dx, dy, dz = x2 - x1, y2 - y1, z2 - z1
    det = dx * dx + dy * dy
    a = (dz * (z3 - z1) + dy * (y3 - y1) + dx * (x3 - x1)) / det

    return [x1 + a * dx, y1 + a * dy, z1 + a * dz]

def normalToPath_3d(start_path, end_path, point):
    """
    Метод расчёта точки нормали к траектории

    :param start_path: начальные координаты пути
    :type start_path: list
    :param end_path: конечные координаты пути
    :type end_path: list
    :param point:   точка в пространстве
    :type point: Point
    :return: нормализованная точка на траектории
    """
    x1,y1,z1 = start_path[0], start_path[1], start_path[2]
    x2, y2, z2 = end_path[0], end_path[1], end_path[2]
    x3,y3, z3 = point[0], point[1], point[2]

    dx, dy, dz = x2 - x1, y2 - y1, z2 - z1
    det = dx * dx + dy * dy
    if det == 0.0:
        return [x1, y1, z1]

    a = (dz * (z3 - z1) + dy * (y3 - y1) + dx * (x3 - x1)) / det
    return [x1 + a * dx, y1 + a * dy, z1 + a * dz]

def summ_vector(a, b):
   """
   Сложение векторов
   :param a: 
   :param b: 
   :return: c
   """
   return (a[0]+b[0], a[1]+b[1], a[2]+b[2])

def list_to_point(a):
    """
    Конвертация массива в point
    :param a:
    :return:
    """
    point = Point()
    point.x = a[0]
    point.y = a[1]
    point.z = a[2]

    return point

def point_to_list(a):
    return (a.x, a.y, a.z)

def point_to_np(a):
    return np.array([a.x, a.y, a.z])

def np_to_point(a):
    point = Point()
    point.x = a[0]
    point.y = a[1]
    point.z = a[2]

    return point


def dist(a, b, flag_2d=False):
	"""
	Дистанция между точками
	:param a:
	:param b:
	:return:
	"""
	c = a - b
	if flag_2d is True:
		c[2] = 0.0

	return np.linalg.norm(c)


def get_yaw_from_quat(data):
	"""
    Возвращаем курс из кратерниона
    :param data:
    :return:
    """
	(roll, pitch, yaw) = tf.transformations.euler_from_quaternion((data.x,
																   data.y,
																   data.z,
																   data.w))
	return yaw

def get_quat_from_yaw(yaw):
    """
    return quaternion from yaw
    """
    quat = tf.transformations.quaternion_from_euler(0.0,0.0, yaw)
    return quat

def euler_from_quaternion(quat):
    """
    return euler angles from quaternion
    """

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))
    return roll, pitch, yaw


