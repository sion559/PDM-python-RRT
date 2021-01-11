import sim
import math
import flib
import time
from geometry_tools import *
from quad_sim import quadsim_P2P
import time
import sys
import datetime


import matplotlib 
import matplotlib.pyplot as plt 


#This entire file is HOMEBREW

#enable to generate new 'boxes.csv', 'pose.csv', and 'targets.csv'.
GENERATE_FILES = False

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

def write_boxes_file(box_list):
    np.savetxt("boxes.csv", box_list, delimiter=",")

def read_boxes_file():
    return np.genfromtxt("boxes.csv", delimiter=",")

def plot_boxes(box_list):
    fig = plt.figure() 
    for box in box_list:
        ax = fig.add_subplot(111) 

        rect1 = matplotlib.patches.Rectangle((box[0], box[1]), 
                                            box[3], box[4], 
                                            color ='green') 
        ax.add_patch(rect1) 

    plt.xlim([-30, 30]) 
    plt.ylim([-30, 30]) 

    plt.show()

def calc_distance(a, b):
    diff = a - b
    return np.sqrt( diff.dot(diff) ) 

def calc_total_distance(start, targets):
    total_distance = calc_distance(start, targets[0])
    print(total_distance)
    if len(targets) > 1:
        for i in range(len(targets) - 1):
            print(total_distance)
            total_distance = calc_distance(targets[i], targets[i+1])
    return total_distance
   
def main():
    obst_count = 68
    targetCount = 5
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
    

    #retrieves the boxes from coppeliasim. This is slow (which is why it is commented out, we read from file instead)
    #obstacle collection
    #obst_list = []
    bbox_list = []
    if GENERATE_FILES:
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
             #obst = Obstacle(obst_pose, obst_size, obst_bbox)
             bbox_list.append(obst_bbox)
             #obst_list.append(obst)
        #we write the boxes to a file to retrieve faster later.
        write_boxes_file(bbox_list)

    bbox_list = read_boxes_file()
    #plot_boxes(bbox_list)


    #target collection
    deliveries = []
    
    for i in range(targetCount):
        err, targ = sim.simxGetObjectHandle(
                clientID, targetPrefix+str(i+1), sim.simx_opmode_blocking)
        tmp = flib.get_pos(clientID, targ)
        print("Target ", i, "Location: ", tmp)
        deliveries.append(np.array([tmp[0],tmp[1],tmp[2]]))
    if GENERATE_FILES:
        np.savetxt("targets.csv", deliveries, delimiter=",")
      
    
    pose = flib.get_pos(clientID, Quadricopter)
    if GENERATE_FILES:
        np.savetxt("pose.csv", pose, delimiter=",")
    
    print("Start position: ", pose)

    print("Total distance: ", calc_total_distance(pose, deliveries))

    #controller object
    pathControl = quadsim_P2P(pose, bbox_list)

    np.savetxt("pose.csv", pose, delimiter=",")
    np.savetxt("targets.csv", np.array(deliveries), delimiter=",")

    #plan route
    before_rrt_t = datetime.datetime.now()
    while not pathControl.plan(deliveries, clientID):
        print("Retrying planning with: max iterations=", pathControl.rrt.max_iter)
        if pathControl.rrt.use_funnel:
            print("search cone angle[Rad]=", pathControl.rrt.searchTheta)
    print("the path is worthy! Calculation took: ", (datetime.datetime.now() - before_rrt_t).total_seconds(), " seconds.")
             
    lastIter = -1
    lastGoal = -1
    
    #start simulation
    pathControl.iterRun_start() 


    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)
    sim.simxSynchronous(clientID,1)
    lastTime = datetime.datetime.now()
    startTime = now = datetime.datetime.now()
    
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
        if pathControl.goalIter != lastGoal:
            now = datetime.datetime.now()
            print("Time of flight: ", (now-startTime).total_seconds(), "[s]")
            lastGoal = pathControl.goalIter
            startTime = now
            
    pathControl.iterRun_stop()
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
    sim.simxFinish(clientID)

if __name__ == '__main__':
    main()
