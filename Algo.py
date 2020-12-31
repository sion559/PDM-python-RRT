from rrt import RRT
from rrt_with_pathsmoothing import path_smoothing
from Quadrotor import Quadrotor
from drone_3d_trajectory_following import quad_sim
from TrajectoryGenerator import TrajectoryGenerator

show_animation = False

# Simulation parameters
g = 9.81
m = 0.2
Ixx = 1
Iyy = 1
Izz = 1
T = 9#7

# Proportional coefficients
Kp_x = 1
Kp_y = 1
Kp_z = 1
Kp_roll = 25
Kp_pitch = 25
Kp_yaw = 25

# Derivative coefficients
Kd_x = 10
Kd_y = 10
Kd_z = 1

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
end = [10,10,1]

for i in range(10):
    rrt = RRT(start=begin, goal=end, zone_Max=[15,15,20], zone_Min=[-20,-20,0], obstacle_list=obstacleList, max_iter=500, expand_dis=3.0, path_resolution=0.5)
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
    
way = path
length = len(way)
x_coeffs = []
y_coeffs = []
z_coeffs = []
    
for i in range(length-1):
    traj = TrajectoryGenerator(way[i], way[(i + 1)], T)
    traj.solve()
    x_coeffs.append(traj.x_c)
    y_coeffs.append(traj.y_c)
    z_coeffs.append(traj.z_c)
    
quad_sim(begin, x_coeffs, y_coeffs, z_coeffs, obstacleList, length)

