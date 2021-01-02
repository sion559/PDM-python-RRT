
"""
Created on Thu Dec  29 14:20:08 2021

@author: Simon van Gemert
"""

import math
from rrt import RRT
from rrt_with_pathsmoothing import path_smoothing
from drone_3d_trajectory_following import sim
from quad_sim import Single_Point2Point

show_animation = False

#list of spherical obstacles
obstacleList = [
        (5, 5, 5, 1),
        (-3, 6, 3, 1),
        (4, -6, 6, 1),
        (-7, 4, 7, 1)
    ]  # [x,y,z,size]

#start pos
begin = [0,0,0]
#end pos
end = [12,5,1]

for i in range(10):
    rrt = RRT(start=begin, goal=end, zone_Max=[15,15,15], zone_Min=[-15,-15,-1], obstacle_list=obstacleList, max_iter=500, expand_dis=3.0, path_resolution=0.5)
    path = rrt.planning(collision=True)
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
    
#quad = sim()
#quad.Simulate(begin, way, obstacleList)

