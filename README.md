# Drone delivery in cities to balconies throughfunneled RRT*

[![standard-readme compliant](https://img.shields.io/badge/readme%20style-standard-brightgreen.svg?style=flat-square)](https://github.com/RichardLitt/standard-readme)

This is the repository for the Planning & Descision making class project of 2020 by S. van Gemert, A. Elferink, N. Duursma, and L. Peeters.
The projects implements an altered version of the RRT* algorithm by [AtsushiSakai](https://github.com/AtsushiSakai/PythonRobotics) and a drone PID controller by [Abhijitmajumdar](https://github.com/abhijitmajumdar/Quadcopter_simulator).
The drone and path are visualized in [Coppeliasim](https://www.coppeliarobotics.com/)

## Table of Contents

- [Install](#install)
- [Usage](#usage)

## Install
1. Make use that the following dependencies are installed.

	- [Coppeliasim](https://www.coppeliarobotics.com/)
	- Python 3.9.x
	- numpy
	- scipy
	- matplotlib

	Dependencies can be easily installed using the requirements.txt
	```
		pip install -r requirements.txt
	```

2. Clone this repository.

## Usage

1. Start Coppeliasim and open 'quadcopter.ttt'
2. To run the simulation open and run the 'coppeliasim-remote.py' in the 'Code' folder.
	- The simulation starts and ends automaticly.
	- You can move the Start and End shapes in Coppeliasim to alter the drones path.
	- You can add additional end point. The need to be numbered consequetivel(1..x) and named with the assigned prefix.
		The number of end points need to edited in python by changing the amount here in 'coppeliasim-remote.py'
	```
		def main():
    			obst_count = 68
    			targetCount = 5  #<--- number of end points
    			obstaclePrefix = 'column'
    			targetPrefix = 'End' #<--- end goal pre-fix
	```
	- The simulation can in very specific cases crash and stop working. In that case, close python first before stopping the simulation in coppeliasim.
3. Preformance data can be generated with the 'results-generator.py'. This operates based on 'boxes.csv', 'pose.csv', and 'targets.csv'.
	- To generate new files enable the option in 'coppeliasim-remote.py'
	```
		#enable to generate new 'boxes.csv', 'pose.csv', and 'targets.csv'.
		GENERATE_FILES = False
	```
	- Data can be processed into graphs with 'data_processing.py' in de 'data' folder
4. RRT* variables can be edditer in the constructor in 'rrt_star.py'
	```
		def __init__(self,
                 	obstacle_list,
                 	searchTheta=3.14/4,
                 	expand_dis=1.5,
                 	path_resolution=0.05,
                 	goal_sample_rate=4,
                 	max_iter=10000,
                 	connect_circle_dist=4,
                 	search_until_max_iter=False,
                 	use_funnel = True):
	```
5. Path following constraints and timing can be adjust in 'quad_sim.py'
	```
		# Constants
		TIME_SCALING = 1.0 # Any positive number(Smaller is faster). 1.0->Real Time, 0.0->Run as fast as possible
		QUAD_DYNAMICS_UPDATE = 0.002 # seconds
		CONTROLLER_DYNAMICS_UPDATE = 0.002 # seconds
		NEXT_GOAL_DISTANCE = 0.5      #distance from current potion to path node neccesary to move to the next path node
		MINIMAL_END_DISTANCE = 0.2  #distance from end goal that indicated succesfull reach
		END_GOAL_VELOCITY = 0.1    #velocity at the end goal the indicates succesfull reach
	```
6. Controller settings can be found in the constructor in 'quad_sim.py'
	```
	# Controller parameters
        self.CONTROLLER_PARAMETERS = {'Motor_limits':[2000,12000],  #4000,12000
                            'Tilt_limits':[-10,10],
                            'Yaw_Control_Limits':[-900,900],
                            'Z_XY_offset':500,
                            'Linear_PID':{'P':[300,300,7000],'I':[0.03,0.03,6],'D':[450,450,4200]},
                            'Linear_To_Angular_Scaler':[1,1,0],
                            'Yaw_Rate_Scaler':0.18,
                            'Angular_PID':{'P':[22000,22000,1500],'I':[0,0,1.2],'D':[12000,12000,0]},
                            }
	```
