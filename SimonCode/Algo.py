
"""
Created on Thu Dec  29 14:20:08 2021

@author: Simon van Gemert
"""

from quad_sim import quadsim_P2P

#list of cubical obstacles, with equal length and position a origen
#TODO: size per as toevoegen, balk met een centerpunt
obstacleList = [
        (3.5, 3.5, 3.5, 1, 2, 2),
        (-3, 6, 4, 1, 2, 1),
        (4, -6, 6, 2, 1, 2),
        (-7, 4, 7, 1, 3, 1),
        (7, 1, 5, 2, 1.5, 1),
        (1, 1, 0, 5, 4, 1)]  # [x,y,z,dx, dy, dz]

#start pos
begin = [0,0,0]

#end pos
end = [8,3,7]

#initiate simulator
sims = quadsim_P2P(begin, obstacleList)

#plan a path from the current position to the goal
while(not sims.plan(end)):
    None
    
#move to the position
sims.run()
