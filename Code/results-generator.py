import sim
import math
import flib
import time
from geometry_tools import *
from quad_sim import quadsim_P2P
import sys
import datetime


import matplotlib 
import matplotlib.pyplot as plt 


#This entire file is HOMEBREW
print("STARTING HERE ----------------------------------------------------------------")

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
    return np.sqrt( diff.dot(diff) ) * 100  #model scale is 1:100

def calc_total_distance(start, targets):
    total_distance = calc_distance(start, targets[0])
    print("Distance start to target 1    ", total_distance) 
    if len(targets) > 1:
        for i in range(len(targets) - 1):
            distance = calc_distance(targets[i], targets[i+1])
            print("Distance target ", i + 1, " to ", i + 2, "    ", distance)
            total_distance = total_distance + calc_distance(targets[i], targets[i+1])
    return total_distance
   
def main():
    bbox_list = read_boxes_file()
    #plot_boxes(bbox_list)

    #target collection
    deliveries = np.genfromtxt("targets.csv", delimiter=",")
    
    
    pose = np.genfromtxt("pose.csv", delimiter=",")
    print("Start position: ", pose)

    print("Total distance: ", calc_total_distance(pose, deliveries))


    data = []
    #plan route
    for i in range(10):
         #controller object
         pathControl = quadsim_P2P(pose, bbox_list)
         while not pathControl.plan(deliveries):
             print("Retrying planning with: max iterations=", pathControl.rrt.max_iter)
             if pathControl.rrt.use_funnel:
                 print("search cone angle[Rad]=", pathControl.rrt.searchTheta)
         #data.append(pathControl.iteration_data)
         name = "data"+str(i) + ".csv"
         np.savetxt(name, pathControl.iteration_data, delimiter=",")

    for i in range(10):
        name = "data"+str(i) + ".csv"
        data.append(np.genfromtxt(name, delimiter=","))
    
    print(data)
    
    # [[[time   nodes iterations] goal 1] loop]


 
if __name__ == '__main__':
    main()
