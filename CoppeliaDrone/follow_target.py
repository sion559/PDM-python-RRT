import sim
import math
import time
import numpy as np
from geometry_tools import *

def get_pos(clientID, QuadricopterT):
    #Obtain position from simulation
    err, position = sim.simxGetObjectPosition(clientID, QuadricopterT, -1, sim.simx_opmode_oneshot_wait)
    return np.array([position[0],position[1],position[2]])

def get_rot(clientID, QuadricopterT):
    #Obtain orientation from simulation
    err, orientation = sim.simxGetObjectOrientation(clientID, QuadricopterT, -1, sim.simx_opmode_oneshot_wait)
    return np.array([orientation[0],orientation[1],orientation[2]])


rad = math.radians

#Specify Drone Target point
target_point = np.array([-3, -3, 3])

#Give Drone starting position
start_pos = np.array([0., 0., 1.])

#Give drone speed
speed = 0.5

drone_pose = []

if __name__=="__main__":
    
    #Initialize system
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

    #Set drone position to start position
    err = sim.simxSetObjectPosition(clientID, Quadricopter, -1,
                                        (start_pos[0],
                                         start_pos[1],
                                         start_pos[2]),
                                        sim.simx_opmode_blocking)
                                        
    #Start simulation
    sim.simxStartSimulation(clientID, sim.simx_opmode_blocking)
    
    #Define obstacles    
    #err, obst_pose = sim.simxGetObjectHandle(clientID, 'City', sim.simx_opmode_blocking)
    
    print("is conneted!!!")
      
    #Obtain target position                        
    pose = get_pos(clientID, QuadricopterT)
    rot = get_rot(clientID, QuadricopterT)

    t = 0
    old_timer = time.time()

    while True:
        dt = time.time() - old_timer
        t += dt
        old_timer = time.time()
        if dt < 0.0001:
            continue

        # Obtain current drone position
        drone_pose = get_pos(clientID, Quadricopter)
        
        #Obtain unit vector pointing from drone towards target
        goal_vec = target_point - drone_pose
        norm_goal_vec = np.linalg.norm(goal_vec)
        
        #Specify error margin (needed for convergence in last state)
        if norm_goal_vec > 0.1: 
            goal_vec = goal_vec / norm_goal_vec
            #Multiply with speed
            goal_vec *= speed
        
            # Update target position and orientation in CoppeliaSim (drone follows target automatically)
            err = sim.simxSetObjectPosition(clientID, QuadricopterT, -1,
                                        (drone_pose[0] + goal_vec[0],
                                         drone_pose[1] + goal_vec[1],
                                         drone_pose[2] + goal_vec[2]),
                                        sim.simx_opmode_blocking)
        else: 
            print("Goal reached")
            sim.simxFinish(clientID)
        
        
                                        
        
        err = sim.simxSetObjectOrientation(clientID, QuadricopterT, -1, (0, 0, rad(0)), sim.simx_opmode_blocking)

        time.sleep(0.01)

    sim.simxFinish(clientID)
