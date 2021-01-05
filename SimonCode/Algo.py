
"""
Created on Thu Dec  29 14:20:08 2021

@author: Simon van Gemert
"""

import math
from rrt_star import RRTStar
from rrt_with_pathsmoothing import path_smoothing
from quad_sim import Single_Point2Point

show_animation = False

#list of spherical obstacles
obstacleList = [
        (5, 5, 5, 1),
        (-3, 6, 3, 1),
        (4, -6, 6, 1),
        (-7, 4, 7, 1)]  # [x,y,z,size]

#start pos
begin = [0,0,0]
#end pos
end = [12,5,1]

for i in range(10):
    rrt = RRTStar(start=begin, goal=end, obstacle_list=obstacleList, max_iter=500, expand_dis=3.0, path_resolution=0.5)
    path = rrt.planning()
    if(path != None):
        break

# Path smoothing
maxIter = 500
smoothedPath = path_smoothing(path, maxIter, obstacleList)

#replace these with the RRT path
way = []
if(len(smoothedPath)> 0):
    way = smoothedPath

#way.reverse()
print(way)
yaw = []
for Y in range(len(way)-1):
    dx = way[Y+1][0] - way[Y][0]
    dy = way[Y+1][1] - way[Y][1]
    yaw.append(math.atan2(dy,dx))
yaw.append(0)

Single_Point2Point(begin, way, yaw)
