import sim
import math
import flib
import time
from geometry_tools import *
from quad_sim import quadsim_P2P
import sys, getopt

rad = math.radians

targer_point = np.array([-9, -10, 5])
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


# if __name__=="__main__":
    
#     #while True:
#     #    err = sim.simxSetObjectPosition(clientID, QuadricopterT, -1,
#     #                                     pos, sim.simx_opmode_blocking)
#     #    err = sim.simxSetObjectOrientation(clientID, QuadricopterT, -1, ori, sim.simx_opmode_blocking)
    
    
#     # t = 0
#     # old_timer = time.time()
#     # while True:
#     #     dt = time.time() - old_timer
#     #     t += dt
#     #     old_timer = time.time()
#     #     if dt < 0.0001:
#     #         continue


#     #     drone_pose = flib.get_pos(clientID, Quadricopter)


#     #     # формируем вектор скорости в направлении целевой точки
#     #     pose = flib.get_pos(clientID, QuadricopterT)
#     #     goal_vec = targer_point - drone_pose
#     #     norm_goal_vec = np.linalg.norm(goal_vec)
#     #     goal_vec = goal_vec / norm_goal_vec
#     #     # print("goal_vec", goal_vec)
#     #     goal_vec *= speed

#     #     # формирование вектора отталкивания

#     #     # Получает позици близжайшего препятствия
#     #     obst_pose, dist_to_obst = get_near_obst(drone_pose, obst_list)
#     #     print("obst_pose", obst_pose, "dist_to_obst", dist_to_obst)

#     #     # rep_vec = np.array([0,0])
#     #     rep_vec = obst_pose - drone_pose
#     #     power = rep_force(dist_to_obst)
#     #     rep_vec = rep_vec * power
#     #     print("rep forces", rep_vec)


#     #     # публикация в сим
#     #     err = sim.simxSetObjectPosition(clientID, QuadricopterT, -1,
#     #                                     (drone_pose[0] + goal_vec[0]-rep_vec[0],
#     #                                      drone_pose[1] + goal_vec[1]-rep_vec[1],
#     #                                      drone_pose[2] + +goal_vec[2]),
#     #                                     sim.simx_opmode_blocking)
#     #     err = sim.simxSetObjectOrientation(clientID, QuadricopterT, -1, (0, 0, rad(0)), sim.simx_opmode_blocking)

#     #     time.sleep(0.01)

#     sim.simxFinish(clientID)
    
def main():
    obst_count = 17
    targetCount = 1
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

    sim.simxStartSimulation(clientID, sim.simx_opmode_blocking)
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
            clientID, 'End1', sim.simx_opmode_blocking)
        tmp = flib.get_pos(clientID, targ)
        print(targ)
        deliveries.append([tmp[0],tmp[1],tmp[2]])
    
    print(deliveries)
        
    pose = flib.get_pos(clientID, QuadricopterT)
    
    #controller object
    pathControl = quadsim_P2P(pose, bbox_list)
    
    if pathControl.plan(deliveries[-1]):
        print("the path is worthy!")

      
    pathControl.iterRun_start()
    print(pathControl.path[0])
    while pathControl.iterRunGo:
        pos, ori = pathControl.iterRun_move()
        #pathControl.display()
        #print("pos = ",pathControl.iterRun_move())
        
        err = sim.simxSetObjectPosition(clientID, QuadricopterT, -1,
                                         pathControl.path[pathControl.pathIter], sim.simx_opmode_blocking)
        err = sim.simxSetObjectPosition(clientID, Quadricopter, -1,
                                         pos, sim.simx_opmode_blocking)
        err = sim.simxSetObjectOrientation(clientID, Quadricopter, -1, ori, sim.simx_opmode_blocking)
    
    sim.simxFinish(clientID)

if __name__ == '__main__':
    main()
