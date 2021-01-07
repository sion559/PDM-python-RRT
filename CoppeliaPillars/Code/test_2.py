import sim
import math
import flib
import time
from geometry_tools import *

rad = math.radians

targer_point = np.array([-9, -10, 5])
speed = 0.5

r_tolerance = 1.0

c_rep = 1.5
r_field = 10.

drone_pose = []

def rep_force(dist_to_obs):
    # type of 1/х
    return c_rep / dist_to_obs - c_rep / r_field


def get_near_obst(current_pose, obst_array):
    """
    возвращаем координату до близжайшего препятсвия
    """

    near_dist = None
    near_pose = None

    for pose in obst_array:
        dist = np.linalg.norm([current_pose[0] - pose[0], current_pose[1] - pose[1]])
        if near_dist is None or dist < near_dist:
            near_dist = dist
            near_pose = pose
            continue

    return near_pose, dist


if __name__=="__main__":
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

    obst_count = 14
    obst_list = []

    #obstacles list
    for i in range(obst_count):
        string = 'column'+str(i)
        err, Obst = sim.simxGetObjectHandle(clientID, string, sim.simx_opmode_blocking)
        obst_pose = flib.get_pos(clientID, Obst)
        print(string, "=", obst_pose)
        obst_list.append(obst_pose)


    print("is conneted!!!")
    pose = flib.get_pos(clientID, QuadricopterT)
    rot = flib.get_rot(clientID, QuadricopterT)

    t = 0
    old_timer = time.time()

    while True:
        dt = time.time() - old_timer
        t += dt
        old_timer = time.time()
        if dt < 0.0001:
            continue


        drone_pose = flib.get_pos(clientID, Quadricopter)


        # формируем вектор скорости в направлении целевой точки
        pose = flib.get_pos(clientID, QuadricopterT)
        goal_vec = targer_point - drone_pose
        norm_goal_vec = np.linalg.norm(goal_vec)
        goal_vec = goal_vec / norm_goal_vec
        # print("goal_vec", goal_vec)
        goal_vec *= speed

        # формирование вектора отталкивания

        # Получает позици близжайшего препятствия
        obst_pose, dist_to_obst = get_near_obst(drone_pose, obst_list)
        print("obst_pose", obst_pose, "dist_to_obst", dist_to_obst)

        # rep_vec = np.array([0,0])
        rep_vec = obst_pose - drone_pose
        power = rep_force(dist_to_obst)
        rep_vec = rep_vec * power
        print("rep forces", rep_vec)


        # публикация в сим
        err = sim.simxSetObjectPosition(clientID, QuadricopterT, -1,
                                        (drone_pose[0] + goal_vec[0]-rep_vec[0],
                                         drone_pose[1] + goal_vec[1]-rep_vec[1],
                                         drone_pose[2] + +goal_vec[2]),
                                        sim.simx_opmode_blocking)
        err = sim.simxSetObjectOrientation(clientID, QuadricopterT, -1, (0, 0, rad(0)), sim.simx_opmode_blocking)

        time.sleep(0.01)

    sim.simxFinish(clientID)
