import sim
import math
import flib
import time
from geometry_tools import *
from quad_sim import quadsim_P2P
import time
import sys
import datetime

rad = math.radians

speed = 0.5

r_tolerance = 1.0

c_rep = 1.5
r_field = 10.

drone_pose = []

class Obstacle:

    def __init__(self, pose, size, bbox):
        self.pose = pose
        self.size = size
        self.bbox = bbox

def rep_force(dist_to_obs):
    # type of 1/х
    return c_rep / dist_to_obs - c_rep / r_field


def get_near_obst(current_pose, obst_array):
    """
    возвращаем координату до близжайшего препятсвия
    """

    near_dist = None
    near_pose = None

    for obst in obst_array:
        pose = obst.pose
        dist = np.linalg.norm([current_pose[0] - pose[0], current_pose[1] - pose[1]])
        if near_dist is None or dist < near_dist:
            near_dist = dist
            near_pose = pose
            continue

    return near_pose, dist

def convert_bbox(pos, size):
    #by default it gives the opposite corners of the bbox.
    # by substracting the starting point, it is defined as 
    # (px, py, pz, dx, dy, dz) with d.. the length of the box on that axis. (only axis aligned bboxes are currently allowed)
    new_size = np.empty(6)
    new_size[0] = pos[0] - size[3]
    new_size[1] = pos[1] - size[4]
    new_size[2] = pos[2] - size[5] 
    new_size[3] = size[3] - size[0]
    new_size[4] = size[4] - size[1]
    new_size[5] = size[5] - size[2]

    return new_size

   
def main():
    obst_count = 17
    targetCount = 4
    startName='Start1'
    obstaclePrefix = 'column'
    targetPrefix = 'End'
    
    
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

    if clientID != -1:
        print('Connected to remote API server')
    err, QuadricopterT = sim.simxGetObjectHandle(
        clientID, 'Quadricopter_target', sim.simx_opmode_blocking)
    if err == -1:
        print("No Quadricopter")
    err, Quadricopter = sim.simxGetObjectHandle(
        clientID, 'Quadricopter', sim.simx_opmode_blocking)
    if err == -1:
        print("No Quadricopter")

    #sim.simxStartSimulation(clientID, sim.simx_opmode_blocking)
    # enable the synchronous mode on the client:     
    
    print("is connected!!!")
    
    
    #obstacle collection
    obst_list = []
    bbox_list = []
    #obstacles list
    for i in range(obst_count):
        err, Obst = sim.simxGetObjectHandle(
            clientID, obstaclePrefix+str(i), sim.simx_opmode_blocking)
        if err > 0:
            print("could not retrieve column ", i)
        obst_pose = flib.get_pos(clientID, Obst)
        print("col ", i, "POSE: ", obst_pose)
        obst_size = flib.get_size(clientID, Obst)
        print("col ", i, "SIZE: ", obst_size)
        obst_bbox = convert_bbox(obst_pose, obst_size)
        print("col ", i, "BBOX: ", obst_bbox)
        obst = Obstacle(obst_pose, obst_size, obst_bbox)
        bbox_list.append(obst_bbox)
        obst_list.append(obst)
        
    #target collection
    deliveries = []
    for i in range(targetCount):
        err, targ = sim.simxGetObjectHandle(
            clientID, 'End'+str(i+1), sim.simx_opmode_blocking)
        tmp = flib.get_pos(clientID, targ)
        deliveries.append([tmp[0],tmp[1],tmp[2]])
    
    print(deliveries)
        
    pose = flib.get_pos(clientID, Quadricopter)
    
    #controller object
    pathControl = quadsim_P2P(pose, bbox_list)
    
    #plan route
    while not pathControl.plan(deliveries):
        print("Retrying planning with: max iterations=", pathControl.rrt.max_iter, ", search cone angle[Rad]=", pathControl.rrt.searchTheta)
        
    print("the path is worthy!")
             
    lastIter = -1
    
    #start simulation
    pathControl.iterRun_start() 
    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)
    sim.simxSynchronous(clientID,1)
    lastTime = datetime.datetime.now();
    now = datetime.datetime.now()
    
    while pathControl.iterRunGo:
        
        pos, ori = pathControl.iterRun_move()
        #pathControl.display()
            
        sim.simxSetObjectPosition(clientID, Quadricopter, -1,
                                             pos, sim.simx_opmode_streaming)
        sim.simxSetObjectOrientation(clientID, Quadricopter, -1, ori, sim.simx_opmode_streaming)            
            
        
        if(pathControl.pathIter != lastIter):
            sim.simxSetObjectPosition(clientID, QuadricopterT, -1, pathControl.path[pathControl.goalIter][pathControl.pathIter], sim.simx_opmode_streaming)
            now = datetime.datetime.now()
            print("Time between goals: ", (now-lastTime).total_seconds(), "[s]")
            lastIter = pathControl.pathIter
            lastTime = now
            
    
    
    # for target in deliveries:
    #     if pathControl.plan(target):
    #         print("the path is worthy!")
    #     else:
    #         break
              
    #     print(pathControl.path[0])
    #     while pathControl.iterRunGo:
    #         pos, ori = pathControl.iterRun_move()
    #         #pathControl.display()
            
    #         sim.simxSetObjectPosition(clientID, Quadricopter, -1,
    #                                          pos, sim.simx_opmode_streaming)
    #         sim.simxSetObjectOrientation(clientID, Quadricopter, -1, ori, sim.simx_opmode_streaming)            
            
    #         lastTime = now
    #         now = datetime.datetime.now()
    #         if(pathControl.pathIter != lastIter):
    #             sim.simxSetObjectPosition(clientID, QuadricopterT, -1, pathControl.path[pathControl.pathIter], sim.simx_opmode_streaming)
    #             print("Loop time: ", (lastTime-now).microseconds, "[us]")
    #             lastIter = pathControl.pathIter
            
    pathControl.iterRun_stop()
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
    sim.simxFinish(clientID)

if __name__ == '__main__':
    main()
