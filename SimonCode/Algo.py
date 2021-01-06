
"""
Created on Thu Dec  29 14:20:08 2021

@author: Simon van Gemert
"""

from rrt_star import RRTStar
from quad_sim import quadsim_P2P

#list of cubical obstacles, with equal length and position a origen
#TODO: size per as toevoegen, balk met een centerpunt
obstacleList = [
        (40, 40, 40, 2, 2, 2),
        (-30, 60, 30, 1, 2, 1),
        (40, -60, 60, 2, 1, 2),
        (-70, 40, 70, 1, 3, 1),
        (70, 20, 10, 2, 1.5, 1)]  # [x,y,z,dx, dy, dz]

#start pos
begin = [0,0,0]

#end pos
end = [3,3,3]

#initiate simulator
sims = quadsim_P2P(begin, obstacleList)

#plan a path from the current position to the goal
while(not sims.plan(end)):
    None
    
#move to the position
sims.run()
