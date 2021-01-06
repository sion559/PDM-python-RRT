import quadcopter,gui,controller
import signal
import sys
import argparse
import math
from rrt_star import RRTStar

#https://github.com/abhijitmajumdar/Quadcopter_simulator

# Constants
TIME_SCALING = 1.0 # Any positive number(Smaller is faster). 1.0->Real Time, 0.0->Run as fast as possible
QUAD_DYNAMICS_UPDATE = 0.002 # seconds
CONTROLLER_DYNAMICS_UPDATE = 0.005 # seconds
run = True


class quadsim_P2P:
    def __init__(self, start, obstacles):
        self.start = start
        self.obs = obstacles
        self.rrtIter = 1000
        
        self.QUADCOPTER={'q1':{'position':start,'orientation':[0,0,0],'L':0.175,'r':0.0665,'prop_size':[8,3.8],'weight':0.5, 'motorWeight':0.035}}
        # Controller parameters
        self.CONTROLLER_PARAMETERS = {'Motor_limits':[2000,12000],  #4000,12000
                            'Tilt_limits':[-10,10],
                            'Yaw_Control_Limits':[-900,900],
                            'Z_XY_offset':500,
                            #'Linear_PID':{'P':[1,1,23.33]*300,'I':[0.01,0.01,1.112]*4,'D':[3,3,33]*150},
                            'Linear_PID':{'P':[290,290,6000],'I':[0.04,0.04,4.8],'D':[410,410,5000]},
                            'Linear_To_Angular_Scaler':[1,1,0],
                            'Yaw_Rate_Scaler':0.18,
                            'Angular_PID':{'P':[22000,22000,1500],'I':[0,0,1.2],'D':[12000,12000,0]},
                            }
    
        # Catch Ctrl+C to stop threads
        signal.signal(signal.SIGINT, signal_handler)
        # Make objects for quadcopter, gui and controller
        self.quad = quadcopter.Quadcopter(self.QUADCOPTER)
        self.gui_object = gui.GUI(quads=self.QUADCOPTER, obs=obstacles)
        self.display()
        self.ctrl = controller.Controller_PID_Point2Point(self.quad.get_state,self.quad.get_time,self.quad.set_motor_speeds,params=self.CONTROLLER_PARAMETERS,quad_identifier='q1')
                
        
    def run(self):
        yaw = self.List_Natural_Yaw();
        # Start the threads
        self.quad.start_thread(dt=QUAD_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
        self.ctrl.start_thread(update_rate=CONTROLLER_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
        
        # Update the GUI while switching between destination poitions
        for i in range(len(self.path)):          
            self.ctrl.update_target(self.path[i])
            self.ctrl.update_yaw_target(yaw[i])
            print("Goal = ", self.path[i])
            #print("yl = ", len(yaw), " pl ", len(self.path))
            while(self.dist(self.quad.get_position('q1'), self.path[i]) > 1):
                self.display()
        
        vel = 1;
        while(vel > 0 and self.dist(self.quad.get_position('q1'), self.path[-1]) > 0.2):
            vel_t = self.quad.get_linear_rate('q1')
            vel = vel_t[0]**2 + vel_t[1]**2 + vel_t[2]**2
            self.display()
                
        self.quad.stop_thread()
        self.ctrl.stop_thread()
        
    def display(self):
        self.gui_object.quads['q1']['position'] = self.quad.get_position('q1')
        self.gui_object.quads['q1']['orientation'] = self.quad.get_orientation('q1')
        self.gui_object.update()
        
    def place(self, pos):
        self.quad.set_position(pos)
        
    def reset(self):
        self.quad.set_position(self.start)
        
    def plan(self, goal):
        begin = self.quad.get_position('q1')
        rrt = RRTStar(start=begin, goal=goal, obstacle_list=self.obs, max_iter=self.rrtIter, expand_dis=3.0, path_resolution=0.3)
        path = rrt.planning()
        if(path == None):
            #self.rrtIter += 500
            return False
        
        path.reverse()
        self.path = path
        return True
    
    def dist(self, A, B):
        dx = B[0] - A[0]
        dy = B[1] - A[1]
        dz = B[2] - A[2]
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def List_Natural_Yaw(self):
        yaw = []
        for Y in range(len(self.path)-1):
            dx = self.path[Y+1][0] - self.path[Y][0]
            dy = self.path[Y+1][1] - self.path[Y][1]
            yaw.append(math.atan2(dy,dx))
        yaw.append(0)
        return yaw


def Single_Point2Point(start, path, yaw):
    # Set goals to go to
    GOALS = path
    YAWS = yaw
    # Define the quadcopters
    QUADCOPTER={'q1':{'position':start,'orientation':[0,0,0],'L':0.3,'r':0.1,'prop_size':[10,4.5],'weight':1.2}}
    # Controller parameters
    CONTROLLER_PARAMETERS = {'Motor_limits':[4000,9000],
                        'Tilt_limits':[-10,10],
                        'Yaw_Control_Limits':[-900,900],
                        'Z_XY_offset':500,
                        'Linear_PID':{'P':[200,200,6000],'I':[0.04,0.04,4.5],'D':[450,450,5000]},
                        'Linear_To_Angular_Scaler':[1,1,0],
                        'Yaw_Rate_Scaler':0.18,
                        'Angular_PID':{'P':[22000,22000,1500],'I':[0,0,1.2],'D':[12000,12000,0]},
                        }

    # Catch Ctrl+C to stop threads
    signal.signal(signal.SIGINT, signal_handler)
    # Make objects for quadcopter, gui and controller
    quad = quadcopter.Quadcopter(QUADCOPTER)
    gui_object = gui.GUI(quads=QUADCOPTER)
    ctrl = controller.Controller_PID_Point2Point(quad.get_state,quad.get_time,quad.set_motor_speeds,params=CONTROLLER_PARAMETERS,quad_identifier='q1')
    # Start the threads
    quad.start_thread(dt=QUAD_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
    ctrl.start_thread(update_rate=CONTROLLER_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
    # Update the GUI while switching between destination poitions
    while(run==True):
        for goal,y in zip(GOALS,YAWS):
            ctrl.update_target(goal)
            ctrl.update_yaw_target(y)
            while(dist(quad.get_position('q1'), goal) > 0.5):
                gui_object.quads['q1']['position'] = quad.get_position('q1')
                gui_object.quads['q1']['orientation'] = quad.get_orientation('q1')
                gui_object.update()
            
    quad.stop_thread()
    ctrl.stop_thread()

def Multi_Point2Point():
    # Set goals to go to
    GOALS_1 = [(-5,-15,4),(1,1,2)]
    GOALS_2 = [(1,-1,2),(-1,1,4)]
    # Define the quadcopters
    QUADCOPTERS={'q1':{'position':[1,0,4],'orientation':[0,0,0],'L':0.3,'r':0.1,'prop_size':[10,4.5],'weight':1.2},
        'q2':{'position':[-1,0,4],'orientation':[0,0,0],'L':0.15,'r':0.05,'prop_size':[6,4.5],'weight':0.7}}
    # Controller parameters
    CONTROLLER_1_PARAMETERS = {'Motor_limits':[4000,9000],
                        'Tilt_limits':[-10,10],
                        'Yaw_Control_Limits':[-900,900],
                        'Z_XY_offset':500,
                        'Linear_PID':{'P':[300,300,7000],'I':[0.04,0.04,4.5],'D':[450,450,5000]},
                        'Linear_To_Angular_Scaler':[1,1,0],
                        'Yaw_Rate_Scaler':0.18,
                        'Angular_PID':{'P':[22000,22000,1500],'I':[0,0,1.2],'D':[12000,12000,0]},
                        }
    CONTROLLER_2_PARAMETERS = {'Motor_limits':[4000,9000],
                        'Tilt_limits':[-10,10],
                        'Yaw_Control_Limits':[-900,900],
                        'Z_XY_offset':500,
                        'Linear_PID':{'P':[300,300,7000],'I':[0.04,0.04,4.5],'D':[450,450,5000]},
                        'Linear_To_Angular_Scaler':[1,1,0],
                        'Yaw_Rate_Scaler':0.18,
                        'Angular_PID':{'P':[22000,22000,1500],'I':[0,0,1.2],'D':[12000,12000,0]},
                        }

    # Catch Ctrl+C to stop threads
    signal.signal(signal.SIGINT, signal_handler)
    # Make objects for quadcopter, gui and controllers
    gui_object = gui.GUI(quads=QUADCOPTERS)
    quad = quadcopter.Quadcopter(quads=QUADCOPTERS)
    ctrl1 = controller.Controller_PID_Point2Point(quad.get_state,quad.get_time,quad.set_motor_speeds,params=CONTROLLER_1_PARAMETERS,quad_identifier='q1')
    ctrl2 = controller.Controller_PID_Point2Point(quad.get_state,quad.get_time,quad.set_motor_speeds,params=CONTROLLER_2_PARAMETERS,quad_identifier='q2')
    # Start the threads
    quad.start_thread(dt=QUAD_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
    ctrl1.start_thread(update_rate=CONTROLLER_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
    ctrl2.start_thread(update_rate=CONTROLLER_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
    # Update the GUI while switching between destination poitions
    while(run==True):
        for goal1,goal2 in zip(GOALS_1,GOALS_2):
            ctrl1.update_target(goal1)
            ctrl2.update_target(goal2)
            for i in range(150):
                for key in QUADCOPTERS:
                    gui_object.quads[key]['position'] = quad.get_position(key)
                    gui_object.quads[key]['orientation'] = quad.get_orientation(key)
                gui_object.update()
    quad.stop_thread()
    ctrl1.stop_thread()
    ctrl2.stop_thread()

def Single_Velocity():
    # Set goals to go to
    GOALS = [(0.5,0,2),(0,0.5,2),(-0.5,0,2),(0,-0.5,2)]
    # Define the quadcopters
    QUADCOPTER={'q1':{'position':[0,0,0],'orientation':[0,0,0],'L':0.3,'r':0.1,'prop_size':[10,4.5],'weight':1.2}}
    # Controller parameters
    CONTROLLER_PARAMETERS = {'Motor_limits':[4000,9000],
                        'Tilt_limits':[-10,10],
                        'Yaw_Control_Limits':[-900,900],
                        'Z_XY_offset':500,
                        'Linear_PID':{'P':[2000,2000,7000],'I':[0.25,0.25,4.5],'D':[50,50,5000]},
                        'Linear_To_Angular_Scaler':[1,1,0],
                        'Yaw_Rate_Scaler':0.18,
                        'Angular_PID':{'P':[22000,22000,1500],'I':[0,0,1.2],'D':[12000,12000,0]},
                        }

    # Catch Ctrl+C to stop threads
    signal.signal(signal.SIGINT, signal_handler)
    # Make objects for quadcopter, gui and controller
    quad = quadcopter.Quadcopter(QUADCOPTER)
    gui_object = gui.GUI(quads=QUADCOPTER)
    ctrl = controller.Controller_PID_Velocity(quad.get_state,quad.get_time,quad.set_motor_speeds,params=CONTROLLER_PARAMETERS,quad_identifier='q1')
    # Start the threads
    quad.start_thread(dt=QUAD_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
    ctrl.start_thread(update_rate=CONTROLLER_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
    # Update the GUI while switching between destination poitions
    while(run==True):
        for goal in GOALS:
            ctrl.update_target(goal)
            for i in range(150):
                gui_object.quads['q1']['position'] = quad.get_position('q1')
                gui_object.quads['q1']['orientation'] = quad.get_orientation('q1')
                gui_object.update()
    quad.stop_thread()
    ctrl.stop_thread()

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

if __name__ == "__main__":
    #args = parse_args()
    #if args.time_scale>=0: TIME_SCALING = args.time_scale
    #if args.quad_update_time>0: QUAD_DYNAMICS_UPDATE = args.quad_update_time
   # if args.controller_update_time>0: CONTROLLER_DYNAMICS_UPDATE = args.controller_update_time
    #if args.sim == 'single_p2p':
    #    Single_Point2Point()
    #elif args.sim == 'multi_p2p':
    #    Multi_Point2Point()
    #elif args.sim == 'single_velocity':
        Single_Point2Point()
