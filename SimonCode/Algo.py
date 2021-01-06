
"""
Created on Thu Dec  29 14:20:08 2021

@author: Simon van Gemert
"""

import math
from rrt_star import RRTStar
from quad_sim import Single_Point2Point
from quad_sim import quadsim_P2P

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

#reverse path order
path.reverse()

sims = quadsim_P2P(begin)
sims.run(path)
#Single_Point2Point(begin, path, yaw)
