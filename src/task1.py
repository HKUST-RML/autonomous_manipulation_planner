#!/usr/bin/env python
import os
import time
import re
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random
from framework import *
from math import *
from jerk import *
from std_msgs.msg import String
from jerk import *
from shapely.geometry import LineString
from shapely.geometry import Polygon
from shapely.geometry import Point
from shapely.geometry import LinearRing

accuracy = 0.004

def update_points(num):
    global total_object_pose_trajectory, total_finger_position_trajectory, iteration
    if num<60:
        return line, gripper, vertex_edge 
    if iteration<len(total_object_pose_trajectory):
        vertex_edge.set_data([j[0] for j in total_object_pose_trajectory[iteration]+[total_object_pose_trajectory[iteration][0]]], [j[1] for j in total_object_pose_trajectory[iteration]+[total_object_pose_trajectory[iteration][0]]])
        gripper.set_data([total_finger_position_trajectory[iteration][0][0],total_finger_position_trajectory[iteration][1][0]], [total_finger_position_trajectory[iteration][0][1],total_finger_position_trajectory[iteration][1][1]])
        iteration+=1
    return line, gripper, vertex_edge

if __name__ == '__main__':
    global total_object_pose_trajectory, total_finger_position_trajectory, iteration
    iteration = 0
    fig = plt.figure()
    ax = fig.add_subplot(111, autoscale_on=False, xlim=(-0.1, 3.1), ylim=(-0.1, 3.1))
    ax.set_aspect('equal')
    #ax.grid(ls="--")
    environment = [[0,0], [3,0]]
    line, = ax.plot([i[0] for i in environment], [i[1] for i in environment], 'darkgoldenrod', lw=2)
    initial_object_vertex_position = [[0.5,0], [1.2,0], [1.2,1.5], [0.5,1.5]]
    initial_CoM_position = [0.85, 0.75]
    vertex_edge, = ax.plot([i[0] for i in initial_object_vertex_position+[initial_object_vertex_position[0]]], [i[1] for i in initial_object_vertex_position+[initial_object_vertex_position[0]]], '-g', lw=3)
    initial_gripper_finger_position = [[1,2], [1.8,2]]
    [ini_F1, ini_F2] = initial_gripper_finger_position
    gripper, = ax.plot([ini_F1[0],ini_F2[0]], [ini_F1[1],ini_F2[1]], '-or', lw=2, markersize=5)
    target_object_vertex_position = [[1.0784462756331488, 1.6893654271085455], [1.2, 1.0], [2.677211629518312, 1.2604722665003956], [2.5556579051514605, 1.9498376936089412]]
    target_finger_position=None
    try:
        total_object_pose_trajectory, total_finger_position_trajectory = framework(initial_object_vertex_position, initial_CoM_position, initial_gripper_finger_position, target_object_vertex_position, target_finger_position, environment, 'Type 1', mu_gripper=0.2, mu_environment=0.21)
        while True:
            if_continue = raw_input("continue?")
            if if_continue != '':
                break
        ani = animation.FuncAnimation(fig, update_points, np.arange(0, 5000, 1), interval=200, blit=True) 
        plt.show() 
    except:
        print 'Infeasible'


