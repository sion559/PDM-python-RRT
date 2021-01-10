import quadcopter,gui,controller
import argparse
import math
from rrt_star import RRTStar
import datetime
import numpy as np

#original auther:
#https://github.com/abhijitmajumdar/Quadcopter_simulator

# Constants
TIME_SCALING = 1.0 # Any positive number(Smaller is faster). 1.0->Real Time, 0.0->Run as fast as possible
QUAD_DYNAMICS_UPDATE = 0.002 # seconds
CONTROLLER_DYNAMICS_UPDATE = 0.002 # seconds
NEXT_GOAL_DISTANCE = 0.5      #distance from current potion to path node neccesary to move to the next path node
MINIMAL_END_DISTANCE = 0.2  #distance from end goal that indicated succesfull reach
END_GOAL_VELOCITY = 0.1    #velocity at the end goal the indicates succesfull reach


#moved functionality
class quadsim_P2P:
    """
    Class for RRT planning and quadrotor PID control
    
    two main operational modes are available, iterative and automatic
    
    iterative: 
        start a new iterative run with iterRun_start()
        create a loop somewhere else in code and call on iterRun_move() in the loop 
        iterRun_stop stops the run prematurely.
        check on the completion with iterRunGo()
    Automatic:
        call on the autoRun(). the function returns after the drone reaches the end goal
        
    path planning:
        before any run a path needs to be planned
        call on plan(goal) to start RRT*
        returned path is stored internally. if the planning fails the search area is widened for the next call.
    
    """
    #moved functionallity
    def __init__(self, start, obstacles):
        
        """
        initializer
        
        start: 3d-vector with the starting coordinates of the drone
        obestacles: list of 6d-vectors with the rectangular obstacles. format{pos_x, pos_y, pos_z, len_x, len_y, len_z}
        """
        
        self.start = start
        self.obs = obstacles
        self.path = []
        self.iteration_data = []
        self.planReady = False
        self.iterRunGo = False
        self.pathIter = 0
        self.goalIter = 0
        
        self.QUADCOPTER={'q1':{'position':start,'orientation':[0,0,0],'L':0.175,'r':0.0665,'prop_size':[8,3.8],'weight':0.5, 'motorWeight':0.035}}
        # Controller parameters
        self.CONTROLLER_PARAMETERS = {'Motor_limits':[2000,12000],  #4000,12000
                            'Tilt_limits':[-10,10],
                            'Yaw_Control_Limits':[-900,900],
                            'Z_XY_offset':500,
                            #'Linear_PID':{'P':[1,1,23.33]*300,'I':[0.01,0.01,1.112]*4,'D':[3,3,33]*150},
                            'Linear_PID':{'P':[300,300,7000],'I':[0.03,0.03,6],'D':[450,450,4200]},
                            #'Linear_PID':{'P':[300,300,6100],'I':[0.055,0.055,7],'D':[400,400,6500]},
                            #'Linear_PID':{'P':[200,200,4000],'I':[0.05,0.05,7],'D':[300,300,7000]},
                            'Linear_To_Angular_Scaler':[1,1,0],
                            'Yaw_Rate_Scaler':0.18,
                            'Angular_PID':{'P':[22000,22000,1500],'I':[0,0,1.2],'D':[12000,12000,0]},
                            }
    
        # Make objects for quadcopter, gui and controller
        self.quad = quadcopter.Quadcopter(self.QUADCOPTER)
        self.gui_object = gui.GUI(quads=self.QUADCOPTER, obs=obstacles)
        self.ctrl = controller.Controller_PID_Point2Point(self.quad.get_state,self.quad.get_time,self.quad.set_motor_speeds,params=self.CONTROLLER_PARAMETERS,quad_identifier='q1')
        self.rrt = RRTStar(obstacle_list=self.obs) 
      
    #HOMEBREW
    def iterRun_start(self):
        """
        iterRun_start
        use to start an iterative run
        
        """
        if not self.planReady or self.iterRunGo:
            print("controller start invallid")
            return
        
        #self.yaw = self.List_Natural_Yaw();
        # Start the threads
        self.quad.start_thread(dt=QUAD_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
        self.ctrl.start_thread(update_rate=CONTROLLER_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
        self.iterRunGo = True
        self.goalIter = 0
        self.pathIter = 0
        
        print("controller started")
        
    #HOMEBREW
    def iterRun_move(self):
        """
        iterRun_move
        call to excecute 1 iteration of the quadcopter controller run
        
        Returns 'None' in case of a faulty call; plan(goal) and iterRun_start() must have been called before
        Returns quadrotor currect position after succesfull call
        """
        
        if not self.planReady or not self.iterRunGo:
            print("cannot iterate")
            return None
        
        #print("move now")
        
        #calculate the constants for this iteration
        vel_t = self.quad.get_linear_rate('q1')
        vel = vel_t[0]**2 + vel_t[1]**2 + vel_t[2]**2
        pos = self.quad.get_position('q1')
        dist = self.dist(pos, self.path[self.goalIter][self.pathIter])
        pLen = len(self.path[self.goalIter])
        
        #move to the next path node if close enough to the current
        if self.pathIter < pLen-1:
            if dist <= NEXT_GOAL_DISTANCE:
                self.pathIter +=1
                print("Going to goal[", self.pathIter, "] = ", self.path[self.goalIter][self.pathIter])
                self.ctrl.update_target(self.path[self.goalIter][self.pathIter])
                self.ctrl.update_yaw_target(self.yaw[self.pathIter])
        #force full stop at the end goal
        elif self.pathIter == pLen-1:
            if vel <= END_GOAL_VELOCITY and dist <= MINIMAL_END_DISTANCE:
                print("Readched end goal[", self.goalIter, "] = ", self.path[self.goalIter][self.pathIter])
                self.goalIter += 1
                
                #stop is the last goal has been reached
                if self.goalIter >= len(self.path):
                    self.iterRunGo = False
                    return pos, self.quad.get_orientation('q1')
                
                self.yaw = self.List_Natural_Yaw();
                self.pathIter = 0
                
        return pos, self.quad.get_orientation('q1')
        
    #HOMEBREW
    def iterRun_stop(self):
        """
        iterRun_stop
        call to stop the currect run. This cancels the simulation entirely
        """
        self.quad.stop_thread()
        self.ctrl.stop_thread()
        self.pathReady = False
        self.iterRunGo = False
        print("controller stopped")
        
    #rearanged functionallity
    def autoRun(self):
        """
        autoRun
        automatic run simulation loop
        """
        if not self.planReady:
            return
        
        yaw = self.List_Natural_Yaw();
        # Start the threads
        self.quad.start_thread(dt=QUAD_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
        self.ctrl.start_thread(update_rate=CONTROLLER_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
        
        # Update the GUI while switching between destination poitions
        for i in range(len(self.path)):          
            self.ctrl.update_target(self.path[i])
            self.ctrl.update_yaw_target(yaw[i])
            print("Going to goal = ", self.path[i])
            #print("yl = ", len(yaw), " pl ", len(self.path))
            while(self.dist(self.quad.get_position('q1'), self.path[i]) > 1):
                self.display()
        
        vel = 1;
        #force full stop at the end goal
        while(vel > 0 and self.dist(self.quad.get_position('q1'), self.path[-1]) > 0.2):
            vel_t = self.quad.get_linear_rate('q1')
            vel = vel_t[0]**2 + vel_t[1]**2 + vel_t[2]**2
            self.display()
                
        self.quad.stop_thread()
        self.ctrl.stop_thread()
        self.pathReady = False
    
    #moved functionallity
    def display(self):
        """
        display
        disply the drones current position in a 2d graph enviroument
        """
        self.gui_object.quads['q1']['position'] = self.quad.get_position('q1')
        self.gui_object.quads['q1']['orientation'] = self.quad.get_orientation('q1')
        self.gui_object.update()
     
    #HOMEBREW
    def place(self, pos):
        """
        place
        set the drone at the give position
        
        pos: 3d-vector with new drone position
        """
        self.quad.set_position(pos)
    
    #HOMEBREW   
    def reset(self):
        """
        reset
        set the drone to the start position
        """
        self.quad.set_position(self.start)
        self.pathIter = 0
     
    #HOMEBREW
    def plan(self, goal, clientID = 0):
        """
        plan
        use RRT* to plan the path for the drone to follow.
        
        Returns 'False' if no valid path can be found. the search window is automaticly enlarged after a failed search
        Returns 'True' when it is found. the path is also stored internally
        
        goal: 3d-vector with the goal position
        """
        
        if self.planReady: 
            print("a path already excists")
            return True
        
        if self.goalIter == 0 :
            end = self.quad.get_position('q1')
        else:
            end = goal[self.goalIter-1]
        
        while self.goalIter < len(goal):
            before_rrt_t = datetime.datetime.now() 
            begin = end
            end = goal[self.goalIter]
            print("Planning for end goal: ", self.goalIter)
            self.rrt.prePlan(begin, end)
            path = self.rrt.planning(clientID)
            if(path == None):
                #widen search zone with larger cone and more iterations
                self.rrt.searchTheta += 0.3
                self.rrt.max_iter += 1000
                return False
            self.path.append(self.zoomPath(path, 3))

            self.iteration_data.append( np.array([(datetime.datetime.now() - before_rrt_t).total_seconds(),
                                        self.rrt.nodes,
                                        self.rrt.nr_iterations]))
            print("the path is worthy! Calculation took: ", (datetime.datetime.now() - before_rrt_t).total_seconds(), " seconds.")
            
            self.goalIter += 1
            
        self.planReady = True
        self.pathIter = 0
        self.goalIter = 0
        
        self.yaw = self.List_Natural_Yaw();
        
        return True
    
    #HOMEBREW
    def zoomPath(self, path, zoom):
        """
        zoomPath
        create a number of nodes between nodes of an existing path
        
        path: existing path
        zoom: number of new nodes to add plus 1
        """
        new_path = []
        for i in range(len(path)-1):
            new_path.append(path[i])
            for a in range(1,zoom):
                dx = (path[i+1][0] - path[i][0])*a/zoom
                dy = (path[i+1][1] - path[i][1])*a/zoom
                dz = (path[i+1][2] - path[i][2])*a/zoom
                new_path.append([dx+path[i][0], dy+path[i][1], dz+path[i][2]])
        return new_path
    
    #HOMEBREW
    def dist(self, A, B):
        """
        dist
        calculate the distance between two points ||b-a||
        
        A: 3d point a
        B: 2d point b
        """
        dx = B[0] - A[0]
        dy = B[1] - A[1]
        dz = B[2] - A[2]
        return math.sqrt(dx**2 + dy**2 + dz**2)

    #HOMEBREW
    def List_Natural_Yaw(self):
        """
        list_Natural_Yaw
        create a list of yaw values based on the path ends with yaw = 0
        
        Returns yaw value list
        """
        yaw = []
        for Y in range(len(self.path[self.goalIter])-1):
            dx = self.path[self.goalIter][Y+1][0] - self.path[self.goalIter][Y][0]
            dy = self.path[self.goalIter][Y+1][1] - self.path[self.goalIter][Y][1]
            yaw.append(math.atan2(dy,dx))
        yaw.append(0)
        return yaw

#ORIGINAL
def parse_args():
    parser = argparse.ArgumentParser(description="Quadcopter Simulator")
    parser.add_argument("--sim", help='single_p2p, multi_p2p or single_velocity', default='single_p2p')
    parser.add_argument("--time_scale", type=float, default=-1.0, help='Time scaling factor. 0.0:fastest,1.0:realtime,>1:slow, ex: --time_scale 0.1')
    parser.add_argument("--quad_update_time", type=float, default=0.0, help='delta time for quadcopter dynamics update(seconds), ex: --quad_update_time 0.002')
    parser.add_argument("--controller_update_time", type=float, default=0.0, help='delta time for controller update(seconds), ex: --controller_update_time 0.005')
    return parser.parse_args()