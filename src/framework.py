#!/usr/bin/env python
import rospy
import os
import sys
import time
import re
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random
import gol
from general_functions import *
from top_and_middle_layer import *
from math import *
from jerk import *
from std_msgs.msg import String
from jerk import *
from shapely.geometry import LineString
from shapely.geometry import Polygon
from shapely.geometry import Point
from shapely.geometry import LinearRing

accuracy = 0.004

def object_pose_plan_consider_move_in_air(initial_object_vertex_position, target_object_vertex_position, action_sequence, contact_state_trajectory, environment, repeating_time):
    initial_contact_state = obtain_contact_state(initial_object_vertex_position, environment)
    target_contact_state = obtain_contact_state(target_object_vertex_position, environment)
    if initial_contact_state == ['no_contact']:
        for direction in [[0,-1], [-1,0], [0,1], [1,0]]:
            for i in np.arange(0,5,0.01):
                temp_new_vertex_position = Move_in_air_next_vertex_position(initial_object_vertex_position, [direction, i], environment)
                new_contact_state = obtain_contact_state(temp_new_vertex_position, environment)
                if new_contact_state != ['no_contact']:
                    contact_state_after_ini = new_contact_state
                    action_achieve_after_ini = 'Move-in-air'
                    action_parameter_after_ini = [direction, i]
                    object_pose_after_ini = temp_new_vertex_position
                    break
            else:
                continue
            break
    if target_contact_state == ['no_contact']:
        for direction in [[0,-1], [-1,0], [0,1], [1,0]]:
            for i in np.arange(0,5,0.01):
                temp_new_vertex_position = Move_in_air_next_vertex_position(target_object_vertex_position, [direction, i], environment)
                new_contact_state = obtain_contact_state(temp_new_vertex_position, environment)
                if new_contact_state != ['no_contact']:
                    contact_state_before_end= new_contact_state
                    action_achieve_before_end = 'Move-in-air'
                    action_parameter_before_end = [[-direction[0], -direction[1]], i]
                    object_pose_before_end = temp_new_vertex_position
                    break
            else:
                continue
            break
    if initial_contact_state != ['no_contact'] and target_contact_state != ['no_contact']:
        action_parameter_sequence, object_pose_trajectory = object_pose_plan(initial_object_vertex_position, None, action_sequence, contact_state_trajectory, environment, repeating_time)
        last_action_sequence, last_contact_state_trajectory, last_action_parameter_sequence, last_object_pose_trajectory = object_pose_plan_within_contact_state(object_pose_trajectory[-1], target_object_vertex_position, environment)
        action_sequence+=last_action_sequence
        contact_state_trajectory+=last_contact_state_trajectory
        action_parameter_sequence+=last_action_parameter_sequence
        object_pose_trajectory+= last_object_pose_trajectory
    elif initial_contact_state == ['no_contact'] and target_contact_state == ['no_contact']: 
        action_parameter_sequence, object_pose_trajectory = object_pose_plan(object_pose_after_ini, None, action_sequence, contact_state_trajectory, environment, repeating_time)
        last_action_sequence, last_contact_state_trajectory, last_action_parameter_sequence, last_object_pose_trajectory = object_pose_plan_within_contact_state(object_pose_trajectory[-1], object_pose_before_end, environment)
        action_sequence+=last_action_sequence
        contact_state_trajectory+=last_contact_state_trajectory
        action_parameter_sequence+=last_action_parameter_sequence
        object_pose_trajectory+= last_object_pose_trajectory  
        action_sequence = ['Move-in-air']+action_sequence+['Move-in-air']
        contact_state_trajectory = [contact_state_after_ini]+contact_state_trajectory+[['no_contact']]
        object_pose_trajectory = [object_pose_after_ini] + object_pose_trajectory + [target_object_vertex_position]
        action_parameter_sequence = [action_parameter_after_ini] + action_parameter_sequence + [action_parameter_before_end]   
    elif initial_contact_state == ['no_contact'] and target_contact_state != ['no_contact']: 
        action_parameter_sequence, object_pose_trajectory = object_pose_plan(object_pose_after_ini, None, action_sequence, contact_state_trajectory, environment, repeating_time)
        last_action_sequence, last_contact_state_trajectory, last_action_parameter_sequence, last_object_pose_trajectory = object_pose_plan_within_contact_state(object_pose_trajectory[-1], target_object_vertex_position, environment)
        action_sequence+=last_action_sequence
        contact_state_trajectory+=last_contact_state_trajectory
        action_parameter_sequence+=last_action_parameter_sequence
        object_pose_trajectory+= last_object_pose_trajectory 
        action_sequence = ['Move-in-air']+action_sequence
        contact_state_trajectory = [contact_state_after_ini]+contact_state_trajectory 
        object_pose_trajectory = [object_pose_after_ini] + object_pose_trajectory
        action_parameter_sequence = [action_parameter_after_ini] + action_parameter_sequence     
    elif initial_contact_state != ['no_contact'] and target_contact_state == ['no_contact']: 
        action_parameter_sequence, object_pose_trajectory = object_pose_plan(initial_object_vertex_position, None, action_sequence, contact_state_trajectory, environment, repeating_time)
        last_action_sequence, last_contact_state_trajectory, last_action_parameter_sequence, last_object_pose_trajectory = object_pose_plan_within_contact_state(object_pose_trajectory[-1], object_pose_before_end, environment)
        action_sequence+=last_action_sequence
        contact_state_trajectory+=last_contact_state_trajectory
        action_parameter_sequence+=last_action_parameter_sequence
        object_pose_trajectory+= last_object_pose_trajectory
        action_sequence = action_sequence+['Move-in-air']
        contact_state_trajectory = contact_state_trajectory+[['no_contact']]  
        object_pose_trajectory = object_pose_trajectory + [target_object_vertex_position]
        action_parameter_sequence = action_parameter_sequence + [action_parameter_before_end]   
    return action_sequence, contact_state_trajectory, action_parameter_sequence, object_pose_trajectory

def framework(initial_object_vertex_position, initial_CoM_position, initial_gripper_finger_position, target_object_vertex_position, target_finger_position, environment, gripper_type, mu_gripper, mu_environment):
    if gripper_type == 'Type 1':
        bottom_layer=__import__('bottom_layer_gripper1')
    else:
        bottom_layer=__import__('bottom_layer_gripper2')
    gol.set_value('mu_environment', mu_environment)
    gol.set_value('mu_gripper', mu_gripper)
    action_sequence_memory = []
    contact_state_trajectory_memory = []
    initial_contact_state = obtain_contact_state(initial_object_vertex_position, environment)
    target_contact_state = obtain_contact_state(target_object_vertex_position, environment)
    grasp_index_list_memory = dict()
    if initial_contact_state == ['no_contact']:
        for direction in [[0,-1], [-1,0], [0,1], [1,0]]:
            for i in np.arange(0,5,0.01):
                temp_new_vertex_position = Move_in_air_next_vertex_position(initial_object_vertex_position, [direction, i], environment)
                new_contact_state = obtain_contact_state(temp_new_vertex_position, environment)
                if new_contact_state != ['no_contact']:
                    contact_state_after_ini = new_contact_state
                    action_achieve_after_ini = 'Move-in-air'
                    action_parameter_after_ini = [direction, i]
                    object_pose_after_ini = temp_new_vertex_position
                    break
            else:
                continue
            break
    if target_contact_state == ['no_contact']:
        for direction in [[0,-1], [-1,0], [0,1], [1,0]]:
            for i in np.arange(0,5,0.01):
                temp_new_vertex_position = Move_in_air_next_vertex_position(target_object_vertex_position, [direction, i], environment)
                new_contact_state = obtain_contact_state(temp_new_vertex_position, environment)
                if new_contact_state != ['no_contact']:
                    contact_state_before_end= new_contact_state
                    action_achieve_before_end = 'Move-in-air'
                    action_parameter_before_end = [[-direction[0], -direction[1]], i]
                    object_pose_before_end = temp_new_vertex_position
                    break
            else:
                continue
            break
                    
    top_layer_iteration = 0
    contact_state0 = contact_state(environment, initial_object_vertex_position)
    while top_layer_iteration<30:
        if top_layer_iteration != 0:
            contact_state_trajectory_temp = [initial_contact_state]+contact_state_trajectory
            for i in range(len(contact_state_trajectory_temp)-1):
                for contact_state_key, contact_state_cost in contact_state0.action_cost.iteritems():
                    if tuple(sorted(contact_state_trajectory_temp[i]))==tuple(sorted(contact_state_key)):
                        for j in range(len(contact_state_cost)):
                            if tuple(sorted(contact_state0.next_contact_state[contact_state_key][j]))==tuple(sorted(contact_state_trajectory_temp[i+1])):
                                contact_state0.action_cost[contact_state_key][j]+=1
        if initial_contact_state != ['no_contact'] and target_contact_state != ['no_contact']:
            action_sequence, contact_state_trajectory = contact_state0.contact_state_plan(initial_contact_state, target_contact_state)
        elif initial_contact_state == ['no_contact'] and target_contact_state == ['no_contact']:
            action_sequence, contact_state_trajectory = contact_state0.contact_state_plan(contact_state_after_ini, contact_state_before_end)
        elif initial_contact_state == ['no_contact'] and target_contact_state != ['no_contact']:
            action_sequence, contact_state_trajectory = contact_state0.contact_state_plan(contact_state_after_ini, target_contact_state)
        elif initial_contact_state != ['no_contact'] and target_contact_state == ['no_contact']:
            action_sequence, contact_state_trajectory = contact_state0.contact_state_plan(initial_contact_state, contact_state_before_end)
        if action_sequence in action_sequence_memory and contact_state_trajectory in contact_state_trajectory_memory:
            continue 
        last_big_circ_action_sequence=action_sequence[:]
        action_sequence_memory.append(action_sequence[:])
        last_big_circ_contact_state_trajectory=contact_state_trajectory[:]
        contact_state_trajectory_memory.append(contact_state_trajectory[:])
        print action_sequence
        print contact_state_trajectory

        repeating_time_kind_A = 0
        repeating_time_kind_B = 0
        while repeating_time_kind_A<4:
            print 'repeating_time_kind_A', repeating_time_kind_A
            print 'repeating_time_kind_B', repeating_time_kind_B
            time0 = time.time()
            if repeating_time_kind_A>0:
                action_sequence = last_big_circ_action_sequence[:]
                contact_state_trajectory = last_big_circ_contact_state_trajectory[:]
            #try:
            try:
                if repeating_time_kind_B==0:
                    action_sequence, contact_state_trajectory, action_parameter_sequence, object_pose_trajectory = object_pose_plan_consider_move_in_air(initial_object_vertex_position, target_object_vertex_position, action_sequence, contact_state_trajectory, environment, repeating_time_kind_A)
                    old_action_sequence=action_sequence[:]
                    old_contact_state_trajectory=contact_state_trajectory[:]
                    old_action_parameter_sequence=action_parameter_sequence[:]
                    old_object_pose_trajectory=object_pose_trajectory[:]                   
                elif repeating_time_kind_B<3: 
                    old_action_sequence=action_sequence[:]
                    old_contact_state_trajectory=contact_state_trajectory[:]
                    old_action_parameter_sequence=action_parameter_sequence[:]
                    old_object_pose_trajectory=object_pose_trajectory[:] 
                    action_sequence = old_action_sequence[0:unsuitable_action_index]+[old_action_sequence[unsuitable_action_index]]+[old_action_sequence[unsuitable_action_index]]+old_action_sequence[(unsuitable_action_index+1)::]
                    contact_state_trajectory = old_contact_state_trajectory[0:unsuitable_action_index]+[old_contact_state_trajectory[unsuitable_action_index]]+[old_contact_state_trajectory[unsuitable_action_index]]+old_contact_state_trajectory[(unsuitable_action_index+1)::]
                    temp_action_parameter_sequence = old_action_parameter_sequence[:]
                    temp_object_pose_trajectory = old_object_pose_trajectory[:]
                    if old_action_sequence[unsuitable_action_index]=='Tip':
                        action_parameter_sequence = temp_action_parameter_sequence[0:unsuitable_action_index]+[[temp_action_parameter_sequence[unsuitable_action_index][0], temp_action_parameter_sequence[unsuitable_action_index][1], temp_action_parameter_sequence[unsuitable_action_index][2]/2.0]]+[[temp_action_parameter_sequence[unsuitable_action_index][0], temp_action_parameter_sequence[unsuitable_action_index][1], temp_action_parameter_sequence[unsuitable_action_index][2]/2.0]]+temp_action_parameter_sequence[(unsuitable_action_index+1)::]
                        object_pose_trajectory = temp_object_pose_trajectory[0:unsuitable_action_index]+[Tip_next_vertex_position(temp_object_pose_trajectory[unsuitable_action_index-1], action_parameter_sequence[unsuitable_action_index], environment)]+temp_object_pose_trajectory[(unsuitable_action_index)::]
                    elif old_action_sequence[unsuitable_action_index]=='Push':
                        action_parameter_sequence = temp_action_parameter_sequence[0:unsuitable_action_index]+[[temp_action_parameter_sequence[unsuitable_action_index][0], temp_action_parameter_sequence[unsuitable_action_index][1]/2.0]]+[[temp_action_parameter_sequence[unsuitable_action_index][0], temp_action_parameter_sequence[unsuitable_action_index][1]/2.0]]+temp_action_parameter_sequence[(unsuitable_action_index+1)::]
                        object_pose_trajectory = temp_object_pose_trajectory[0:unsuitable_action_index]+[Push_next_vertex_position(temp_object_pose_trajectory[unsuitable_action_index-1], action_parameter_sequence[unsuitable_action_index], environment)]+temp_object_pose_trajectory[(unsuitable_action_index)::]
                    elif old_action_sequence[unsuitable_action_index]=='Tilting-slide':
                        action_parameter_sequence = temp_action_parameter_sequence[0:unsuitable_action_index]+[[temp_action_parameter_sequence[unsuitable_action_index][0], temp_action_parameter_sequence[unsuitable_action_index][1]/2.0]]+[[temp_action_parameter_sequence[unsuitable_action_index][0], temp_action_parameter_sequence[unsuitable_action_index][1]/2.0]]+temp_action_parameter_sequence[(unsuitable_action_index+1)::]
                        object_pose_trajectory = temp_object_pose_trajectory[0:unsuitable_action_index]+[Tilting_slide_next_vertex_position(temp_object_pose_trajectory[unsuitable_action_index-1], action_parameter_sequence[unsuitable_action_index], environment)]+temp_object_pose_trajectory[(unsuitable_action_index)::]
                else:
                    repeating_time_kind_B=0
                    repeating_time_kind_A+=1  
                    continue          
            except:
                repeating_time_kind_B=0
                repeating_time_kind_A+=1
                continue 
            print action_sequence, contact_state_trajectory, action_parameter_sequence, object_pose_trajectory
            object_pose_trajectory_plus_ini = [initial_object_vertex_position]+object_pose_trajectory
            grasp_index_list = [(i,j) for i in range(1,101) for j in range(1,101)]
            grasp_index_list.append((0,0))
            feasible_grasp_set_list, grasp_index_list_memory = bottom_layer.obtain_feasible_grasp_set_of_each_pose(grasp_index_list, object_pose_trajectory_plus_ini, environment, grasp_index_list_memory)
            suitable_grasp_list_sequence = bottom_layer.obtain_suitable_grasp_set_of_each_action(action_sequence, action_parameter_sequence, object_pose_trajectory_plus_ini, feasible_grasp_set_list, initial_object_vertex_position, initial_CoM_position, environment)
            print time.time()-time0
            unsuitable_action_index_set = []
            for i in range(len(action_sequence)):
                if suitable_grasp_list_sequence[i] == []:
                    unsuitable_action_index_set.append(i)
            unsuitable_grasp_move_in_air = False
            for j in unsuitable_action_index_set:
                if action_sequence[j]=='Move-in-air':
                    unsuitable_grasp_move_in_air = True
                    break
            if unsuitable_grasp_move_in_air ==True:
                repeating_time_kind_B=0
                repeating_time_kind_A+=1
                continue    
            elif unsuitable_action_index_set !=[]:  
                unsuitable_action_index = min(unsuitable_action_index_set)  
                print unsuitable_action_index 
                repeating_time_kind_B+=1
                continue
            try:
                total_finger_position_trajectory, total_object_pose_trajectory = bottom_layer.bottom_layer_grasp_plan(initial_gripper_finger_position[0], initial_gripper_finger_position[1], initial_CoM_position, action_sequence, action_parameter_sequence, object_pose_trajectory_plus_ini, feasible_grasp_set_list, suitable_grasp_list_sequence, environment, accuracy=0.002)
                return total_object_pose_trajectory, total_finger_position_trajectory  
            except:
                repeating_time_kind_B=0
                repeating_time_kind_A+=1
                continue 
            continue         
        top_layer_iteration+=1
    return 'Infeasible fff'            
