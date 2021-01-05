"""

Path planning Sample Code with RRT with path smoothing

@author: AtsushiSakai(@Atsushi_twi)

"""

import math
import os
import random
import sys

import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    from rrt import RRT
except ImportError:
    raise

show_animation = True

#eddited for 3d
def get_path_length(path):
    le = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        dz = path[i + 1][2] - path[i][2]
        d = math.sqrt(dx * dx + dy * dy + dz*dz)
        le += d

    return le

#eddited for 3d
def get_target_point(path, targetL):
    le = 0
    ti = 0
    lastPairLen = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        dz = path[i + 1][2] - path[i][2]
        d = math.sqrt(dx * dx + dy * dy + dz*dz)
        le += d
        if le >= targetL:
            ti = i - 1
            lastPairLen = d
            break

    try:
        partRatio = (le - targetL) / lastPairLen
    except ZeroDivisionError:
        partRatio = 1
    

    x = path[ti][0] + (path[ti + 1][0] - path[ti][0]) * partRatio
    y = path[ti][1] + (path[ti + 1][1] - path[ti][1]) * partRatio
    z = path[ti][2] + (path[ti + 1][2] - path[ti][2]) * partRatio

    return [x, y, z, ti]


#eddited for 3d
def line_collision_check(first, second, obstacleList):
    # Line Equation

    dx = second[0] - first[0]
    dy = second[1] - first[1]
    dz = second[2] - first[2]
    try:
        d = math.hypot(math.hypot(dx,dy), dz)
    except ZeroDivisionError:
        return False
    lx = dx/d
    ly = dy/d
    lz = dz/d
    a = lx**2 + ly**2 + lz**2

    #use ABC formula to find number of solutions for n in len(n*norm(r)) = size
    for (ox, oy, oz, size) in obstacleList:
        c = ox**2 + oy**2 + oz**2 - size**2
        b = (lx * ox + ly * oy + lz * oz)*2
        D = b**2 - 4*a*c
        if(D >= 0):   #1 or 2 solutions, thus collision
            return False

    """try:
        a = second[1] - second[1]
        b = -(second[0] - first[0])
        c = second[1] * (second[0] - first[0]) - second[0] * (second[1] - first[1])
    except ZeroDivisionError:
        return False

    for (ox, oy, oz, size) in obstacleList:
        d = abs(a * ox + b * oy + c) / (math.sqrt(a * a + b * b))
        if d <= size:
            return False"""

    return True  # OK

#eddited for 3d
def path_smoothing(path, max_iter, obstacle_list):
    le = get_path_length(path)

    for i in range(max_iter):
        # Sample two points
        pickPoints = [random.uniform(0, le), random.uniform(0, le)]
        pickPoints.sort()
        first = get_target_point(path, pickPoints[0])
        second = get_target_point(path, pickPoints[1])

        if first[3] <= 0 or second[3] <= 0:
            continue

        if (second[3] + 1) > len(path):
            continue

        if second[3] == first[3]:
            continue

        # collision check
        if not line_collision_check(first, second, obstacle_list):
            continue

        # Create New path
        newPath = []
        newPath.extend(path[:first[3] + 1])
        newPath.append([first[0], first[1], first[2]])
        newPath.append([second[0], second[1], second[2]])
        newPath.extend(path[second[3] + 1:])
        path = newPath
        le = get_path_length(path)

    return path
