
import quadcopter,gui,controller
import signal
import argparse
import math


import sim
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../CoppeliaDrone/")

#https://github.com/abhijitmajumdar/Quadcopter_simulator

# Constants
TIME_SCALING = 1.0 # Any positive number(Smaller is faster). 1.0->Real Time, 0.0->Run as fast as possible
QUAD_DYNAMICS_UPDATE = 0.002 # seconds
CONTROLLER_DYNAMICS_UPDATE = 0.005 # seconds
run = True


class quadsim_P2P:
    def __init__(self, start, simHandle, objectHandle):
        self.start = start
        self.simHandle = simHandle
        self.objectHandle = objectHandle
        
        self.QUADCOPTER={'q1':{'position':start,'orientation':[0,0,0],'L':0.175,'r':0.0665,'prop_size':[8,3.8],'weight':0.5, 'motorWeight':0.035}}
        # Controller parameters
        self.CONTROLLER_PARAMETERS = {'Motor_limits':[4000,9000],
                            'Tilt_limits':[-10,10],
                            'Yaw_Control_Limits':[-900,900],
                            'Z_XY_offset':500,
                            'Linear_PID':{'P':[290,290,6700],'I':[0.04,0.04,4.8],'D':[410,410,5000]},
                            'Linear_To_Angular_Scaler':[1,1,0],
                            'Yaw_Rate_Scaler':0.18,
                            'Angular_PID':{'P':[22000,22000,1500],'I':[0,0,1.2],'D':[12000,12000,0]},
                            }
    
        # Catch Ctrl+C to stop threads
        signal.signal(signal.SIGINT, signal_handler)
        # Make objects for quadcopter, gui and controller
        self.quad = quadcopter.Quadcopter(self.QUADCOPTER)
        self.gui_object = gui.GUI(quads=self.QUADCOPTER)
        self.ctrl = controller.Controller_PID_Point2Point(self.quad.get_state,self.quad.get_time,self.quad.set_motor_speeds,params=self.CONTROLLER_PARAMETERS,quad_identifier='q1')
                
        
    def run(self, path):
        yaw = self.List_Natural_Yaw(path);
        # Start the threads
        self.quad.start_thread(dt=QUAD_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
        self.ctrl.start_thread(update_rate=CONTROLLER_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
        
        # Update the GUI while switching between destination poitions
        for i in range(len(path)):          
            self.ctrl.update_target(path[i])
            self.ctrl.update_yaw_target(yaw[i])
            print("Goal = ", path[i])
            while(self.dist(self.quad.get_position('q1'), path[i]) > 1):
                sim.simxSetObjectPosition(self.simHandle, self.objectHandle, -1, self.quad.get_position('q1'), sim.simx_Opmode_non_blocking)
        
        vel = 1;
        while(vel > 0 and self.dist(self.quad.get_position('q1'), path[-1]) > 0.2):
            vel_t = self.quad.get_linear_rate('q1')
            vel = vel_t[0]**2 + vel_t[1]**2 + vel_t[2]**2
            sim.simxSetObjectPosition(self.simHandle, self.objectHandle, -1, self.quad.get_position('q1'), sim.simx_Opmode_non_blocking)
            
        self.quad.stop_thread()
        self.ctrl.stop_thread()
        
    def place(self, pos):
        self.quad.set_position(pos)
        
    def reset(self):
        self.quad.set_position(self.start)
        
    def List_Natural_Yaw(path):
        yaw = []
        for Y in range(len(path)-1):
            dx = path[Y+1][0] - path[Y][0]
            dy = path[Y+1][1] - path[Y][1]
            yaw.append(math.atan2(dy,dx))
        yaw.append(0)
        return yaw

    def dist(A, B):
        dx = B[0] - A[0]
        dy = B[1] - A[1]
        dz = B[2] - A[2]
        return math.sqrt(dx**2 + dy**2 + dz**2)

 
def parse_args():
    parser = argparse.ArgumentParser(description="Quadcopter Simulator")
    parser.add_argument("--sim", help='single_p2p, multi_p2p or single_velocity', default='single_p2p')
    parser.add_argument("--time_scale", type=float, default=-1.0, help='Time scaling factor. 0.0:fastest,1.0:realtime,>1:slow, ex: --time_scale 0.1')
    parser.add_argument("--quad_update_time", type=float, default=0.0, help='delta time for quadcopter dynamics update(seconds), ex: --quad_update_time 0.002')
    parser.add_argument("--controller_update_time", type=float, default=0.0, help='delta time for controller update(seconds), ex: --controller_update_time 0.005')
    return parser.parse_args()

def signal_handler(signal, frame):
    global run
    run = False
    print('Stopping')
    sys.exit(0)