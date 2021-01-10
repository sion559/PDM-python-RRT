# -*- coding: utf-8 -*-

import math
import numpy as np

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

    return (val[0][0], val[1][0], 0.0)


def normalize_angle(ang):
    """
    Нормализуем угол от -pi до pi
    :param ang:
    :return:
    """
    while ang > math.pi:
        ang -= 2.0 * math.pi
    while ang < -math.pi:
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

def dist_to_lind (point, _point1, _point2):
    """
    возращаем дистанцию до линии
    :param point:
    :param _point1:
    :param _point2:
    :return:
    """
    a = (_point2[0]-_point1[0])*(point[1]-_point1[1])-(_point1[1]-_point1[1])*(point[0]-_point1[0])
    b = math.sqrt((_point2[0]-_point1[0])**2+(_point2[1]-_point1[1])**2)
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
