#!/usr/bin/env python
import rospy
import time
import re
import numpy as np
import random
from general_functions import *
from math import *
from jerk import *
from shapely.geometry import LineString
from shapely.geometry import Polygon
from shapely.geometry import Point
from shapely.geometry import LinearRing

def obtain_contact_state(current_vertex_position, profile_environment, current_vertex_position_buffer=None, accuracy = 0.004):
    V=current_vertex_position
    l=[[V[i], V[(i+1)%(len(current_vertex_position))]] for i in range(len(current_vertex_position))]
    E=dict()
    for i in range(len(profile_environment)-1):
        E[i+1] = [profile_environment[len(profile_environment)-i-2], profile_environment[len(profile_environment)-i-1]]
    contact_state_list = []
    if current_vertex_position_buffer == None:
        if Polygon(current_vertex_position).buffer(-accuracy).distance(LineString(profile_environment)) == 0:
            return ['Invalid']
    else:
        if Polygon(current_vertex_position_buffer).distance(LineString(profile_environment)) == 0:
            return ['Invalid']
    if Polygon(current_vertex_position).distance(LineString(profile_environment))>accuracy*2:
        return ['no_contact']
    for i in range(len(current_vertex_position)):
        for E_key in E.iterkeys():
            if Point(V[i]).distance(LineString(E[E_key]))<accuracy and Point(V[(i+1)%(len(current_vertex_position))]).distance(LineString(E[E_key]))<accuracy:
                contact_state_list.append('l'+str(i+1)+'E'+str(E_key))
            else:
                temp_point0 = [V[i][0]+(V[(i+1)%(len(current_vertex_position))][0]-V[i][0])/10*3, V[i][1]+(V[(i+1)%(len(current_vertex_position))][1]-V[i][1])/10*3]
                if Point(V[i]).distance(LineString(E[E_key]))<accuracy and Point(temp_point0).distance(LineString(E[E_key]))<accuracy:
                    contact_state_list.append('l'+str(i+1)+'E'+str(E_key))
                else:
                    temp_point1 = [V[i][0]+(V[(i+1)%(len(current_vertex_position))][0]-V[i][0])/10*7, V[i][1]+(V[(i+1)%(len(current_vertex_position))][1]-V[i][1])/10*7] 
                    if Point(V[(i+1)%(len(current_vertex_position))]).distance(LineString(E[E_key]))<accuracy and Point(temp_point1).distance(LineString(E[E_key]))<accuracy:
                        contact_state_list.append('l'+str(i+1)+'E'+str(E_key))
    for i in range(len(current_vertex_position)):
        for E_key in E.iterkeys():
            if ('l'+str(i+1)+'E'+str(E_key)) not in contact_state_list and ('l'+str((i-1)%(len(current_vertex_position))+1)+'E'+str(E_key)) not in contact_state_list:
                if Point(V[i]).distance(LineString(E[E_key]))<accuracy:
                    contact_state_list.append('V'+str(i+1)+'E'+str(E_key))
    if len(contact_state_list)==1:
        contact_state_list.append('single_contact')
    elif len(contact_state_list)==2:
        contact_state_list.append('double_contact')
    else:
        return ['Invalid']
    return sorted(contact_state_list)

def whether_contains(principle_contact_state1, principle_contact_state2, object_edge_number):
    if principle_contact_state1 == principle_contact_state2:
        return False
    elif principle_contact_state1[2:4] != principle_contact_state2[2:4]:
        return False
    elif principle_contact_state1[0]=='V' and principle_contact_state2[0]=='l':
        return False
    elif (int(principle_contact_state1[1])+1)%object_edge_number==int(principle_contact_state2[1]):
        return True
    elif int(principle_contact_state1[1])==int(principle_contact_state2[1]):
        return True
    else:
        return False

def whether_less_constraint(contact_state1, contact_state2, object_edge_number):
    temp_contact_state1=contact_state1[:]
    temp_contact_state2=contact_state2[:]
    if sorted(contact_state1)==sorted(contact_state2):
        return False
    if 'single_contact' in temp_contact_state1:
        temp_contact_state1.remove('single_contact')
    if 'single_contact' in temp_contact_state2:
        temp_contact_state2.remove('single_contact')
    if 'double_contact' in temp_contact_state1:
        temp_contact_state1.remove('double_contact')
    if 'double_contact' in temp_contact_state2:
        temp_contact_state2.remove('double_contact')
    if len(temp_contact_state1)<len(temp_contact_state2):
        return False
    satisfying_principle_contact_number = 0
    contain_list=dict()
    for principle_contact in temp_contact_state2:
        if principle_contact in temp_contact_state1:
            satisfying_principle_contact_number+=1
            continue
        else:
            contain_list[principle_contact]=[]
            for principle_contact_state_1 in temp_contact_state1:
                if whether_contains(principle_contact_state_1, principle_contact, object_edge_number)==True:
                    contain_list[principle_contact].append(principle_contact_state_1)
            if len(contain_list[principle_contact])==1:
                satisfying_principle_contact_number+=1
    if satisfying_principle_contact_number == len(temp_contact_state2):
        for principle_contact in temp_contact_state2:
            for principle_contact0 in temp_contact_state2:
                if principle_contact0==principle_contact:
                    continue
                if contain_list.has_key(principle_contact) and contain_list.has_key(principle_contact0) and set(contain_list[principle_contact]).intersection(set(contain_list[principle_contact0]))!=set():
                    return False
        return True
    return False

def calculate_parameter_tip(current_vertex_position, target_contact_state, target_object_pose=None, profile_environment = [[0,5], [0,0], [5,0]], scope = [[-0.1,-0.1], [3.1,-0.1], [3.1,3.1], [-0.1,3.1]], rotate_step = 5, searching_used = False):
    current_contact_state = obtain_contact_state(current_vertex_position, profile_environment)
    contact_list = []
    if target_object_pose != None:
        rotate_step = 1
    ini_rotate_step = rotate_step
    for vertex in current_vertex_position:
        if Point(vertex).distance(LineString(profile_environment))<0.004:
            contact_list.append(vertex)  
    for contact in contact_list:
        for direction in ['CCW', 'CW']:
            old_vertex_position = current_vertex_position
            old_vertex_position_buffer = Polygon(current_vertex_position).buffer(-0.002).boundary.coords[:]
            accumulate_angle = 0
            rotate_step = ini_rotate_step
            iteration = 0
            while True:
                if direction == 'CCW':
                    new_vertex_position = [point_position_after_rotation(vertex, contact, rotate_step) for vertex in old_vertex_position]
                    new_vertex_position_buffer = [point_position_after_rotation(vertex, contact, rotate_step) for vertex in old_vertex_position_buffer]
                else:
                    new_vertex_position = [point_position_after_rotation(vertex, contact, -rotate_step) for vertex in old_vertex_position]
                    new_vertex_position_buffer = [point_position_after_rotation(vertex, contact, -rotate_step) for vertex in old_vertex_position_buffer]
                if target_object_pose != None:
                    if Polygon(new_vertex_position).buffer(-0.01).intersects(LineString(profile_environment)) == True:
                        break
                    accumulate_angle = accumulate_angle+rotate_step
                    old_vertex_position = new_vertex_position
                    i = 0
                    for k in range(len(target_object_pose)):
                        if Point(target_object_pose[k]).distance(Point(new_vertex_position[k]))<0.01:
                            i+=1
                    if i == len(target_object_pose):
                        return contact, direction, accumulate_angle, new_vertex_position
                else:
                    if Polygon(new_vertex_position_buffer).distance(LineString(profile_environment))>0:
                        accumulate_angle = accumulate_angle+rotate_step
                        old_vertex_position = new_vertex_position
                        old_vertex_position_buffer = new_vertex_position_buffer
                        new_contact_state = obtain_contact_state(new_vertex_position, profile_environment, new_vertex_position_buffer)
                        if set(new_contact_state) == set(target_contact_state):
                            return contact, direction, accumulate_angle, new_vertex_position
                        elif searching_used == True and set(new_contact_state) != set(current_contact_state) and new_contact_state!=['Invalid']:        
                            break           
                    else:
                        min_bound = 0
                        max_bound = rotate_step
                        while max_bound-min_bound>0.0001:
                            rotate_step = float(min_bound)+(float(max_bound) - float(min_bound))/2.0
                            if direction == 'CCW':
                                new_vertex_position = [point_position_after_rotation(vertex, contact, rotate_step) for vertex in old_vertex_position]
                                new_vertex_position_buffer = [point_position_after_rotation(vertex, contact, rotate_step) for vertex in old_vertex_position_buffer]
                            else:
                                new_vertex_position = [point_position_after_rotation(vertex, contact, -rotate_step) for vertex in old_vertex_position]
                                new_vertex_position_buffer = [point_position_after_rotation(vertex, contact, -rotate_step) for vertex in old_vertex_position_buffer]
                            new_contact_state = obtain_contact_state(new_vertex_position, profile_environment, new_vertex_position_buffer)
                            if sorted(new_contact_state) == sorted(target_contact_state):
                                accumulate_angle = accumulate_angle+rotate_step
                                return contact, direction, accumulate_angle, new_vertex_position
                            elif searching_used == True and set(new_contact_state) != set(current_contact_state) and new_contact_state!=['Invalid']:        
                                break

                            if Polygon(new_vertex_position_buffer).distance(LineString(profile_environment))>0:
                                min_bound = rotate_step
                                max_bound = max_bound
                            else:
                                min_bound = min_bound
                                max_bound = rotate_step
                        break
    return 'No Solution'

def calculate_parameter_push(current_vertex_position, target_contact_state, target_object_pose = None, profile_environment = [[0,5], [0,0], [5,0]], scope = [[-0.1,-0.1], [3.1,-0.1], [3.1,3.1], [-0.1,3.1]], translate_step = 0.2, searching_used = False, accuracy = 0.004):

    current_contact_state = obtain_contact_state(current_vertex_position, profile_environment)
    contact_list = []
    contact_env_edge_list=dict()
    if target_object_pose != None:
        translate_step = 0.01
    ini_translate_step = translate_step
    for vertex in current_vertex_position:
        if Point(vertex).distance(LineString(profile_environment))<0.004:
            contact_env_edge_list[tuple(vertex)]=[]
            contact_list.append(vertex)
            for j in range(len(profile_environment)):
                if j!=len(profile_environment)-1 and Point(vertex).distance(LineString([profile_environment[j], profile_environment[j+1]])) <accuracy:
                    contact_env_edge_list[tuple(vertex)].append([profile_environment[j], profile_environment[j+1]])
    direction_memory = []
    for contact in contact_list:
        for direction in [[x[1][0]-x[0][0], x[1][1]-x[0][1]] for x in contact_env_edge_list[tuple(contact)]] + [[-x[1][0]+x[0][0], -x[1][1]+x[0][1]] for x in contact_env_edge_list[tuple(contact)]]:
            if direction in direction_memory:
                continue
            direction_memory.append(direction)
            direction = [direction[0]/sqrt(direction[0]**2+direction[1]**2), direction[1]/sqrt(direction[0]**2+direction[1]**2)]
            old_vertex_position = current_vertex_position
            old_vertex_position_buffer = Polygon(current_vertex_position).buffer(-0.002).boundary.coords[:]  
            accumulate_distance = 0
            translate_step = ini_translate_step
            iteration = 0
            while True:  
                if searching_used ==True:
                    iteration +=1
                    if iteration>10:
                        break
                new_vertex_position = [[vertex[0]+translate_step*direction[0], vertex[1]+translate_step*direction[1]] for vertex in old_vertex_position]
                new_vertex_position_buffer = [[vertex[0]+translate_step*direction[0], vertex[1]+translate_step*direction[1]] for vertex in old_vertex_position_buffer]
                if target_object_pose != None:
                    if Polygon(new_vertex_position_buffer).distance(LineString(profile_environment))<=0:
                        break
                    if Polygon(new_vertex_position).distance(LineString(profile_environment))>0.05:
                        break
                    accumulate_distance = accumulate_distance+translate_step
                    old_vertex_position = new_vertex_position
                    old_vertex_position_buffer = new_vertex_position_buffer
                    i = 0
                    for k in range(len(target_object_pose)):
                        if Point(target_object_pose[k]).distance(Point(new_vertex_position[k]))<0.01:
                            i+=1
                    if i == len(target_object_pose):
                        return direction, accumulate_distance, new_vertex_position
                else:
                    
                    if Polygon(new_vertex_position_buffer).distance(LineString(profile_environment))>0:
                        
                        if Polygon(new_vertex_position).within(Polygon(scope)) ==False:
                            break
                        accumulate_distance = accumulate_distance+translate_step
                        old_vertex_position = new_vertex_position
                        old_vertex_position_buffer = new_vertex_position_buffer
                        
                        new_contact_state = obtain_contact_state(new_vertex_position, profile_environment, new_vertex_position_buffer)
                        if set(new_contact_state) == set(target_contact_state):
                            return direction, accumulate_distance, new_vertex_position
                        elif searching_used == True and set(new_contact_state) != set(current_contact_state) and new_contact_state!=['Invalid']:     
                            break
                    else:
                        min_bound = 0
                        max_bound = translate_step
                        while max_bound - min_bound >0.0001:
                            translate_step = float(min_bound)+(float(max_bound) - float(min_bound))/2.0
                            new_vertex_position = [[vertex[0]+translate_step*direction[0], vertex[1]+translate_step*direction[1]] for vertex in old_vertex_position]
                            new_vertex_position_buffer = [[vertex[0]+translate_step*direction[0], vertex[1]+translate_step*direction[1]] for vertex in old_vertex_position_buffer]
                            new_contact_state = obtain_contact_state(new_vertex_position, profile_environment, new_vertex_position_buffer)
                            if sorted(new_contact_state) == sorted(target_contact_state):
                                accumulate_distance = accumulate_distance+translate_step
                                return direction, accumulate_distance, new_vertex_position
                            elif searching_used == True and set(new_contact_state) != set(current_contact_state) and new_contact_state!=['Invalid']:   
                                break
                            if Polygon(new_vertex_position_buffer).distance(LineString(profile_environment))>0:
                                min_bound = translate_step
                                max_bound = max_bound
                            else:
                                min_bound = min_bound
                                max_bound = translate_step
                        break
    return 'No Solution'

def Tilting_slide_contact_edge_point(current_vertex_position, target_contact_state, profile_environment):
    current_contact_state = obtain_contact_state(current_vertex_position, profile_environment)
    if 'double_contact' not in current_contact_state:
        return 'Invalid'
    E=dict()
    for i in range(len(profile_environment)-1):
        E[i+1] = [profile_environment[len(profile_environment)-i-2], profile_environment[len(profile_environment)-i-1]]
    current_contact_state.remove('double_contact')
    for i in range(len(current_vertex_position)):
        if re.match('V'+str(i+1), current_contact_state[0])!=None and re.match('V'+str(i+1), current_contact_state[1])!=None:
            return 'Invalid'
    contact_edge = dict()
    contact_edge_point = dict()
    for contact_state_element in current_contact_state:
        if re.match('V', contact_state_element)!=None:
            contact_point = current_vertex_position[int(contact_state_element[1])-1]
            for i in range(len(profile_environment)-1):
                if contact_state_element[3]==str(i+1):
                    contact_edge['E'+str(i+1)] = E[i+1]
                    contact_edge_point['E'+str(i+1)] = contact_point
        else:
            contact_point = [current_vertex_position[int(contact_state_element[1])-1], current_vertex_position[int(contact_state_element[1])%(len(current_vertex_position))]]
            for point in contact_point:
                if Point(point).distance(Point(profile_environment[1]))<0.01:
                    contact_point.remove(point)
            nearest_contact_point = contact_point[0]
            for point in contact_point:
                if Point(point).distance(Point(profile_environment[1]))<Point(nearest_contact_point).distance(Point(profile_environment[1])):
                    nearest_contact_point = point
            contact_point = nearest_contact_point
            for i in range(len(profile_environment)-1):
                if contact_state_element[3]==str(i+1):
                    contact_edge['E'+str(i+1)] = E[i+1]
                    contact_edge_point['E'+str(i+1)] = contact_point
    if 'double_contact' in target_contact_state:
        target_contact_state.remove('double_contact')
    if re.match('V', current_contact_state[0])==None and re.match('V', current_contact_state[1])==None:
        if re.match('V', target_contact_state[0])==None and re.match('V', target_contact_state[1])==None:
            return 'Invalid'       
        else:
            for contact_state_element in target_contact_state:
                if re.match('V', contact_state_element)!=None:
                    contact_point = current_vertex_position[int(contact_state_element[1])-1]
                    for i in range(len(profile_environment)-1):
                        if contact_state_element[3]==str(i+1):
                            if Point(contact_point).distance(LineString(E[i+1]))>0.004:
                                return 'Invalid'
                            contact_edge['E'+str(i+1)] = E[i+1]
                            contact_edge_point['E'+str(i+1)] = contact_point
    if 'double_contact' not in target_contact_state:
        target_contact_state.append('double_contact')
    return contact_edge_point

def Tilting_slide_calculate_next_vertex_position_simplified(contact_edge_point, total_angle, old_vertex_position, profile_environment):
    E=dict()
    E_direction=dict()
    for i in range(len(profile_environment)-1):
        E[i+1] = [profile_environment[len(profile_environment)-i-2], profile_environment[len(profile_environment)-i-1]]
        E_direction[i+1] = [(E[i+1][1][0]-E[i+1][0][0])/Point(E[i+1][0]).distance(Point(E[i+1][1])), (E[i+1][1][1]-E[i+1][0][1])/Point(E[i+1][0]).distance(Point(E[i+1][1]))]
    E_key_set = sorted([i for i in contact_edge_point.iterkeys()])
    Ea_direction = [(E[int(E_key_set[0][-1])][1][0]-E[int(E_key_set[0][-1])][0][0])/Point(E[int(E_key_set[0][-1])][0]).distance(Point(E[int(E_key_set[0][-1])][1])), (E[int(E_key_set[0][-1])][1][1]-E[int(E_key_set[0][-1])][0][1])/Point(E[int(E_key_set[0][-1])][0]).distance(Point(E[int(E_key_set[0][-1])][1]))]
    Eb_direction = [(E[int(E_key_set[1][-1])][0][0]-E[int(E_key_set[1][-1])][1][0])/Point(E[int(E_key_set[1][-1])][0]).distance(Point(E[int(E_key_set[1][-1])][1])), (E[int(E_key_set[1][-1])][0][1]-E[int(E_key_set[1][-1])][1][1])/Point(E[int(E_key_set[1][-1])][0]).distance(Point(E[int(E_key_set[1][-1])][1]))]
    
    intersect_angle_Ea=abs(calculate_intersect_angle_AB_CD(contact_edge_point[E_key_set[1]], contact_edge_point[E_key_set[0]], E[int(E_key_set[0][-1])][0], E[int(E_key_set[0][-1])][1]))
    intersect_angle_Eb=abs(calculate_intersect_angle_AB_CD(contact_edge_point[E_key_set[0]], contact_edge_point[E_key_set[1]], E[int(E_key_set[1][-1])][1], E[int(E_key_set[1][-1])][0]))
    intersect_angleEaEb = abs(calculate_intersect_angle_AB_CD(E[int(E_key_set[1][-1])][0], E[int(E_key_set[1][-1])][1], E[int(E_key_set[0][-1])][0], E[int(E_key_set[0][-1])][1]))
    EaEbcontact_distance = Point(contact_edge_point[E_key_set[1]]).distance(Point(contact_edge_point[E_key_set[0]]))
    target_intersect_angle_Ea = intersect_angle_Ea-total_angle
    target_intersect_angle_Eb = intersect_angle_Eb+total_angle
    Ebdistance = sin(target_intersect_angle_Ea*pi/180)*EaEbcontact_distance/sin(intersect_angleEaEb*pi/180)
    Eadistance = sin(target_intersect_angle_Eb*pi/180)*EaEbcontact_distance/sin(intersect_angleEaEb*pi/180)
    new_Ea_contact_point = [E[int(E_key_set[0][-1])][0][0]+Eadistance*Ea_direction[0], E[int(E_key_set[0][-1])][0][1]+Eadistance*Ea_direction[1]]
    new_Eb_contact_point = [E[int(E_key_set[0][-1])][0][0]+Ebdistance*Eb_direction[0], E[int(E_key_set[0][-1])][0][1]+Ebdistance*Eb_direction[1]]
    delta_translate, D_r, A_r = calculate_parameter_translate_and_rotate(contact_edge_point[E_key_set[0]], contact_edge_point[E_key_set[1]], new_Ea_contact_point, new_Eb_contact_point)
    A_r = A_r*180/pi
    new_vertex_position = [[point_position_after_rotation(vertex, contact_edge_point[E_key_set[0]], A_r)[0]+delta_translate[0], point_position_after_rotation(vertex, contact_edge_point[E_key_set[0]], A_r)[1]+delta_translate[1]] for vertex in old_vertex_position]
    return new_vertex_position

def calculate_parameter_tilting_slide(current_vertex_position, target_contact_state, target_object_pose=None, profile_environment = [[0,5], [0,0], [5,0]], scope = [[-0.1,-0.1], [3.1,-0.1], [3.1,3.1], [-0.1,3.1]], rotate_step = 30, searching_used=False): 
    current_contact_state = obtain_contact_state(current_vertex_position, profile_environment)
    if target_object_pose != None:
        rotate_step = 1
    try:
        contact_edge_point=Tilting_slide_contact_edge_point(current_vertex_position, target_contact_state, profile_environment)
    except:
        return 'No Solution'

    for direction in ['+', '-']:
        A_r = 0
        rotate_step0 = rotate_step
        while True:
            if direction == '+':
                rotate_step_real = abs(rotate_step)
            else:
                rotate_step_real = -abs(rotate_step)
            A_r = A_r + rotate_step_real
            try:
                new_vertex_position = Tilting_slide_calculate_next_vertex_position_simplified(contact_edge_point, A_r, current_vertex_position, profile_environment)
            except:
                return 'No Solution'
            if target_object_pose != None:
                if Polygon(new_vertex_position).buffer(-0.002).crosses(LineString(profile_environment)) == True:
                    break
                i = 0
                for k in range(len(target_object_pose)):
                    if Point(target_object_pose[k]).distance(Point(new_vertex_position[k]))<0.01:
                        i+=1
                if i == len(target_object_pose):
                    return A_r, new_vertex_position
            else:
                if Polygon(new_vertex_position).buffer(-0.002).crosses(LineString(profile_environment)) == False:
                    new_contact_state = obtain_contact_state(new_vertex_position, profile_environment)
                    if set(new_contact_state) == set(target_contact_state):
                        return A_r, new_vertex_position
                    elif searching_used == True and set(new_contact_state) != set(current_contact_state) and new_contact_state!=['Invalid']:      
                        break
                else:
                    min_bound = 0
                    max_bound = rotate_step0
                    while max_bound - min_bound >0.0001:
                        rotate_step0 = float(min_bound)+(float(max_bound) - float(min_bound))/2.0
                        if direction == '+':
                            rotate_angle = A_r-rotate_step_real+rotate_step0
                            try:
                                new_vertex_position = Tilting_slide_calculate_next_vertex_position_simplified(contact_edge_point, rotate_angle, current_vertex_position, profile_environment)
                            except:
                                return 'No Solution'
                        else:
                            rotate_angle = A_r-rotate_step_real-rotate_step0
                            try:
                                new_vertex_position = Tilting_slide_calculate_next_vertex_position_simplified(contact_edge_point, rotate_angle, current_vertex_position, profile_environment)
                            except:
                                return 'No Solution'

                        new_contact_state = obtain_contact_state(new_vertex_position, profile_environment)
                        if sorted(new_contact_state) == sorted(target_contact_state):
                            return rotate_angle, new_vertex_position
                        elif searching_used == True and set(new_contact_state) != set(current_contact_state) and new_contact_state!=['Invalid']: 
                            temp_new_contact_state = new_contact_state[:]
                            if 'double_contact' not in temp_new_contact_state:
                                return 'No Solution'
                            temp_new_contact_state.remove('double_contact')
                            if temp_new_contact_state[0][0]=='V' and temp_new_contact_state[1][0]=='V':
                                break

                        if Polygon(new_vertex_position).buffer(-0.002).crosses(LineString(profile_environment)) == False:
                            min_bound = rotate_step0
                            max_bound = max_bound
                        else:
                            min_bound = min_bound
                            max_bound = rotate_step0
                    break
    return 'No Solution'

def calculate_parameter_move_in_air(current_vertex_position, target_contact_state, target_object_pose = None, moving_direction='up', profile_environment = [[0,5], [0,0], [5,0]], translate_step = 0.2):
    if target_object_pose != None:
        translate_step = 0.01
    ini_translate_step = translate_step
    if moving_direction == 'up':
        direction = [0,1]
    elif moving_direction == 'down':
        direction = [0,-1]
    elif moving_direction == 'left':
        direction = [-1,0]
    elif moving_direction == 'right':
        direction = [1,0]
    old_vertex_position = current_vertex_position
    accumulate_distance = 0
    translate_step = ini_translate_step
    while True:
        new_vertex_position = [[vertex[0]+translate_step*direction[0], vertex[1]+translate_step*direction[1]] for vertex in old_vertex_position]
        if target_object_pose != None:
            if Polygon(new_vertex_position).buffer(-0.002).intersects(LineString(profile_environment)) == True:
                break
            accumulate_distance = accumulate_distance+translate_step
            old_vertex_position = new_vertex_position
            i = 0
            for k in range(len(target_object_pose)):
                if Point(target_object_pose[k]).distance(Point(new_vertex_position[k]))<0.01:
                    i+=1
                if i == len(target_object_pose):
                    return direction, accumulate_distance, new_vertex_position
        else:
            if Polygon(new_vertex_position).buffer(-0.002).crosses(LineString(profile_environment)) == False:
                accumulate_distance = accumulate_distance+translate_step
                old_vertex_position = new_vertex_position
                if set(obtain_contact_state(new_vertex_position, profile_environment)) == set(target_contact_state):
                    return direction, accumulate_distance, new_vertex_position
                else:
                    min_bound = 0
                    max_bound = translate_step
                    while max_bound - min_bound >0.0001:
                        translate_step = float(min_bound)+(float(max_bound) - float(min_bound))/2.0
                        new_vertex_position = [[vertex[0]+translate_step*direction[0], vertex[1]+translate_step*direction[1]] for vertex in old_vertex_position]
                        if set(obtain_contact_state(new_vertex_position, profile_environment)) == set(target_contact_state):
                            accumulate_distance = accumulate_distance+translate_step
                            return direction, accumulate_distance, new_vertex_position
                        if Polygon(new_vertex_position).buffer(-0.002).crosses(LineString(profile_environment)) == False:
                            min_bound = translate_step
                            max_bound = max_bound
                        else:
                            min_bound = min_bound
                            max_bound = translate_step
                    break
    return 'No Solution'

def Tip_next_vertex_position(current_object_vertex_position, action_parameter, profile_environment):
    [rotation_pole, rotation_direction, rotation_angle] = action_parameter
    if rotation_direction=='CCW':
        new_vertex_position = [point_position_after_rotation(vertex, rotation_pole, rotation_angle) for vertex in current_object_vertex_position]
    else:
        new_vertex_position = [point_position_after_rotation(vertex, rotation_pole, -rotation_angle) for vertex in current_object_vertex_position]
    return new_vertex_position

def Push_next_vertex_position(current_object_vertex_position, action_parameter, profile_environment):
    [push_direction, push_distance] = action_parameter
    new_vertex_position = [[vertex[0]+push_distance*push_direction[0], vertex[1]+push_distance*push_direction[1]] for vertex in current_object_vertex_position]
    return new_vertex_position

def Move_in_air_next_vertex_position(current_object_vertex_position, action_parameter, profile_environment):
    [move_direction, move_distance] = action_parameter
    new_vertex_position = [[vertex[0]+move_distance*move_direction[0], vertex[1]+move_distance*move_direction[1]] for vertex in current_object_vertex_position]
    return new_vertex_position

def Tilting_slide_next_vertex_position(current_object_vertex_position, action_parameter, profile_environment):
    [target_contact_state, A_r]=action_parameter
    contact_edge_point=Tilting_slide_contact_edge_point(current_object_vertex_position, target_contact_state, profile_environment)
    new_vertex_position = Tilting_slide_calculate_next_vertex_position_simplified(contact_edge_point, A_r, current_object_vertex_position, profile_environment)
    return new_vertex_position    

class contact_state:
    def __init__(self, environment, initial_vertex_position):
        self.contact_state_list, self.next_contact_state, self.action_to_next_contact_state, self.action_cost = self.contact_state_map(environment, initial_vertex_position)

    def contact_state_map(self, environment, initial_vertex_position, scope = [[-0.1,-0.1], [3.1,-0.1], [3.1,3.1], [-0.1,3.1]]):
        time0 = time.time()
        state_position_pair_state_list = []
        state_position_pair_position_list = []
        for i in range(len(environment)-1):
            env_segment = [environment[i], environment[i+1]]
            env_vec = [(env_segment[1][0]-env_segment[0][0])/LineString(env_segment).length, (env_segment[1][1]-env_segment[0][1])/LineString(env_segment).length] 
            for j in range(len(initial_vertex_position)):
                feasible_length = LineString(env_segment).length-LineString([initial_vertex_position[j], initial_vertex_position[(j+1)%(len(initial_vertex_position))]]).length
                if feasible_length==0:
                    feasible_length_step = feasible_length/10+0.0001
                else:
                    feasible_length_step = feasible_length/10
                for key_vertex_position_index in np.append(np.arange(0, feasible_length, feasible_length_step), feasible_length):
                    key_vertex_position = [env_segment[0][0]+env_vec[0]*key_vertex_position_index, env_segment[0][1]+env_vec[1]*key_vertex_position_index]
                    key_next_vertex_position = [key_vertex_position[0]+env_vec[0]*LineString([initial_vertex_position[j], initial_vertex_position[(j+1)%(len(initial_vertex_position))]]).length, key_vertex_position[1]+env_vec[1]*LineString([initial_vertex_position[j], initial_vertex_position[(j+1)%(len(initial_vertex_position))]]).length]
                    delta_translate, D_r, A_r = calculate_parameter_translate_and_rotate(initial_vertex_position[j], initial_vertex_position[(j+1)%(len(initial_vertex_position))], key_vertex_position, key_next_vertex_position)
                    A_r = A_r*180/pi
                    current_vertex_position = [[point_position_after_rotation(vertex, initial_vertex_position[j], A_r)[0]+delta_translate[0], point_position_after_rotation(vertex, initial_vertex_position[j], A_r)[1]+delta_translate[1]] for vertex in initial_vertex_position]
                    if Polygon(current_vertex_position).buffer(-0.004).intersects(LineString(environment))==False and Polygon(current_vertex_position).within(Polygon(scope)) ==True:
                        current_contact_state = obtain_contact_state(current_vertex_position, environment)  
                        if current_contact_state != ['Invalid'] and current_contact_state != ['no_contact']:
                            state_position_pair_state_list.append(current_contact_state)
                            state_position_pair_position_list.append(current_vertex_position)
                    key_vertex_local_angle = calculate_intersect_angle_AB_CD(initial_vertex_position[j], initial_vertex_position[(j+1)%(len(initial_vertex_position))], initial_vertex_position[j], initial_vertex_position[(j-1)%(len(initial_vertex_position))])
                    feasible_angle = 180-key_vertex_local_angle
                    for random_angle in np.append(np.arange(0, feasible_angle, feasible_angle/25), feasible_angle):
                        key_next_vector = np.dot([[cos(random_angle*pi/180), -sin(random_angle*pi/180)],[sin(random_angle*pi/180), cos(random_angle*pi/180)]], np.expand_dims(env_vec, axis=1)).transpose().tolist()[0]
                        key_to_key_next_length = LineString([initial_vertex_position[j], initial_vertex_position[(j+1)%(len(initial_vertex_position))]]).length
                        key_next_vertex_position = [key_vertex_position[0]+key_next_vector[0]*key_to_key_next_length, key_vertex_position[1]+key_next_vector[1]*key_to_key_next_length]
                        delta_translate, D_r, A_r = calculate_parameter_translate_and_rotate(initial_vertex_position[j], initial_vertex_position[(j+1)%(len(initial_vertex_position))], key_vertex_position, key_next_vertex_position)
                        A_r = A_r*180/pi
                        current_vertex_position = [[point_position_after_rotation(vertex, initial_vertex_position[j], A_r)[0]+delta_translate[0], point_position_after_rotation(vertex, initial_vertex_position[j], A_r)[1]+delta_translate[1]] for vertex in initial_vertex_position]
                        if Polygon(current_vertex_position).buffer(-0.004).intersects(LineString(environment))==False and Polygon(current_vertex_position).within(Polygon(scope)) ==True:
                            current_contact_state = obtain_contact_state(current_vertex_position, environment)  
                            if current_contact_state != ['Invalid'] and current_contact_state != ['no_contact']:
                                state_position_pair_state_list.append(current_contact_state)
                                state_position_pair_position_list.append(current_vertex_position) 
                        else:
                            for direction in ['+', '-']:
                                old_vertex_position_temp = current_vertex_position
                                for iteration in range(30):
                                    if direction == '+':
                                        new_vertex_position_temp = [[vertex[0]+0.01*env_vec[0], vertex[1]+0.01*env_vec[1]] for vertex in old_vertex_position_temp]
                                    else:
                                        new_vertex_position_temp = [[vertex[0]-0.01*env_vec[0], vertex[1]-0.01*env_vec[1]] for vertex in old_vertex_position_temp]
                                    old_vertex_position_temp = new_vertex_position_temp
                                    if Polygon(new_vertex_position_temp).buffer(-0.004).intersects(LineString(environment))==False and Polygon(new_vertex_position_temp).within(Polygon(scope)) ==True:
                                        current_contact_state = obtain_contact_state(new_vertex_position_temp, environment)  
                                        if current_contact_state != ['Invalid'] and current_contact_state != ['no_contact']:
                                            state_position_pair_state_list.append(current_contact_state)
                                            state_position_pair_position_list.append(new_vertex_position_temp)    
                                            break
                                else:
                                    continue
                                break  
        contact_state_list = xlist(list(set(xtuple(state_position_pair_state_list))))
        print contact_state_list
        print len(contact_state_list)
        next_contact_state = dict()
        action_to_next_contact_state = dict()
        action_cost = dict()
        for k in range(len(state_position_pair_position_list)):
            print k
            temp_contact_state_list = contact_state_list
            temp_contact_state_list = [sorted(i) for i in temp_contact_state_list]
            temp_contact_state_list.remove(sorted(state_position_pair_state_list[k]))
            for candidate_contact_state in temp_contact_state_list:
                if whether_less_constraint(state_position_pair_state_list[k], candidate_contact_state, len(initial_vertex_position))==False and whether_less_constraint(candidate_contact_state, state_position_pair_state_list[k], len(initial_vertex_position))==False:
                    continue
                temp_state_position_pair_state_list_k = tuple(sorted(state_position_pair_state_list[k]))
                for action in ['Tip', 'Push', 'Tilting-slide']:
                    if action == 'Tip':
                        if 'double_contact' in temp_state_position_pair_state_list_k and 'double_contact' in candidate_contact_state:
                            continue
                        if next_contact_state.has_key(temp_state_position_pair_state_list_k)==True:
                            already_have = False
                            for k0 in range(len(next_contact_state[temp_state_position_pair_state_list_k])):
                                if next_contact_state[temp_state_position_pair_state_list_k][k0]==tuple(sorted(candidate_contact_state)) and action_to_next_contact_state[temp_state_position_pair_state_list_k][k0]=='Tip':
                                    already_have = True
                                    break
                            if already_have == True:
                                continue
                            if calculate_parameter_tip(state_position_pair_position_list[k], candidate_contact_state, None, environment, searching_used=True) != 'No Solution':
                                next_contact_state[temp_state_position_pair_state_list_k].append(tuple(sorted(candidate_contact_state)))
                                action_to_next_contact_state[temp_state_position_pair_state_list_k].append('Tip')
                                action_cost[temp_state_position_pair_state_list_k].append(1)                     
                        else:
                            if calculate_parameter_tip(state_position_pair_position_list[k], candidate_contact_state, None, environment, searching_used=True) != 'No Solution':
                                next_contact_state[temp_state_position_pair_state_list_k]=[tuple(sorted(candidate_contact_state))]
                                action_to_next_contact_state[temp_state_position_pair_state_list_k]=['Tip']
                                action_cost[temp_state_position_pair_state_list_k]=[1] 
                    elif action == 'Push':
                        if next_contact_state.has_key(temp_state_position_pair_state_list_k)==True:
                            already_have = False
                            for k0 in range(len(next_contact_state[temp_state_position_pair_state_list_k])):
                                if next_contact_state[temp_state_position_pair_state_list_k][k0]==tuple(sorted(candidate_contact_state)) and action_to_next_contact_state[temp_state_position_pair_state_list_k][k0]=='Push':
                                    already_have = True
                                    break
                            if already_have == True:
                                continue
                            if calculate_parameter_push(state_position_pair_position_list[k], candidate_contact_state, None, environment, searching_used=True) != 'No Solution':
                                next_contact_state[temp_state_position_pair_state_list_k].append(tuple(sorted(candidate_contact_state)))
                                action_to_next_contact_state[temp_state_position_pair_state_list_k].append('Push')
                                action_cost[temp_state_position_pair_state_list_k].append(1)
                        else:
                            if calculate_parameter_push(state_position_pair_position_list[k], candidate_contact_state, None, environment, searching_used=True) != 'No Solution':
                                next_contact_state[temp_state_position_pair_state_list_k]=[tuple(sorted(candidate_contact_state))]
                                action_to_next_contact_state[temp_state_position_pair_state_list_k]=['Push']
                                action_cost[temp_state_position_pair_state_list_k]=[1]
 
                    elif action == 'Tilting-slide':
                        if 'double_contact' not in state_position_pair_state_list[k]:
                            continue
                        if 'double_contact' not in candidate_contact_state:
                            continue
                        temp_state_position_pair_state_list = state_position_pair_state_list[k][:]
                        temp_candidate_contact_state = candidate_contact_state[:]
                        temp_state_position_pair_state_list.remove('double_contact')
                        temp_candidate_contact_state.remove('double_contact')
                        if temp_state_position_pair_state_list[0][0:2] == temp_state_position_pair_state_list[1][0:2]:
                            continue
                        if temp_candidate_contact_state[0][0:2] == temp_candidate_contact_state[1][0:2]:
                            continue
                        if next_contact_state.has_key(temp_state_position_pair_state_list_k)==True:
                            already_have = False
                            for k0 in range(len(next_contact_state[temp_state_position_pair_state_list_k])):
                                if next_contact_state[temp_state_position_pair_state_list_k][k0]==tuple(sorted(candidate_contact_state)) and action_to_next_contact_state[temp_state_position_pair_state_list_k][k0]=='Tilting-slide':
                                    already_have = True
                                    break
                            if already_have == True:
                                continue
                            if calculate_parameter_tilting_slide(state_position_pair_position_list[k], candidate_contact_state, None, environment, searching_used=True) != 'No Solution':
                                next_contact_state[temp_state_position_pair_state_list_k].append(tuple(sorted(candidate_contact_state)))
                                action_to_next_contact_state[temp_state_position_pair_state_list_k].append('Tilting-slide')
                                action_cost[temp_state_position_pair_state_list_k].append(1)
                        else:
                            if calculate_parameter_tilting_slide(state_position_pair_position_list[k], candidate_contact_state, None, environment, searching_used=True) != 'No Solution':
                                next_contact_state[tuple(sorted(state_position_pair_state_list[k]))]=[tuple(sorted(candidate_contact_state))]
                                action_to_next_contact_state[tuple(sorted(state_position_pair_state_list[k]))]=['Tilting-slide']
                                action_cost[tuple(sorted(state_position_pair_state_list[k]))]=[1]
 
        return contact_state_list, next_contact_state, action_to_next_contact_state, action_cost

    def contact_state_plan(self, ini_contact_state, final_contact_state):
        contact_state_list_temp = self.contact_state_list[:]
        contact_state_list = []
        for contact_state in contact_state_list_temp:
            contact_state_list.append(tuple(sorted(contact_state)))
        OPEN_set = {tuple(sorted(ini_contact_state))}
        CLOSED = set()
        past_cost=dict.fromkeys(contact_state_list, float('inf'))
        past_cost[tuple(sorted(ini_contact_state))]=0
        parent = dict()
        previous_action = dict()
        while OPEN_set != set():
            min_past_cost = past_cost[list(OPEN_set)[0]]
            first_node = list(OPEN_set)[0]
            for node in list(OPEN_set):
                if past_cost[node]<min_past_cost:
                    min_past_cost = past_cost[node]
                    first_node=node
            OPEN_set.remove(first_node)
            CLOSED.add(first_node)
            if set(first_node) == set(final_contact_state):
                final_node = first_node
                break
            for contact_state, action_list in self.action_to_next_contact_state.iteritems():
                if tuple(sorted(contact_state))==first_node:
                    first_node_action_set = action_list
                    break
            for contact_state, next_contact_state_list in self.next_contact_state.iteritems():
                if tuple(sorted(contact_state))==first_node:
                    first_node_next_contact_list = next_contact_state_list
                    break
            for contact_state, action_cost_list in self.action_cost.iteritems():
                if tuple(sorted(contact_state))==first_node:
                    first_node_action_cost = action_cost_list[:]
                    break
            for i in range(len(first_node_action_set)):
                if tuple(sorted(first_node_next_contact_list[i])) in CLOSED:
                    continue
                tentative_past_cost = past_cost[first_node]+first_node_action_cost[i]
                if tentative_past_cost < past_cost[tuple(sorted(first_node_next_contact_list[i]))]:
                    past_cost[tuple(sorted(first_node_next_contact_list[i]))] = tentative_past_cost
                    parent[tuple(sorted(first_node_next_contact_list[i]))] = first_node
                    previous_action[tuple(sorted(first_node_next_contact_list[i]))] = first_node_action_set[i]
                    OPEN_set.add(tuple(sorted(first_node_next_contact_list[i])))
        previous_action_total = []
        contact_state_trajectory = []
        while final_node != tuple(sorted(ini_contact_state)):
            previous_action_total.append(previous_action[final_node])
            contact_state_trajectory.append(final_node)
            final_node = parent[final_node]
        previous_action_total = previous_action_total[::-1]
        contact_state_trajectory = xlist(contact_state_trajectory[::-1])
        return previous_action_total, contact_state_trajectory

def object_pose_plan(initial_object_vertex_position, final_object_vertex_position, action_list, contact_state_trajectory, profile_environment, repeating_time=0):
    begin_object_vertex_position = initial_object_vertex_position
    action_parameter = []
    object_pose_trajectory = []
    if repeating_time != 0 and len(action_list)<=1:
        return 'Infeasible'
    if repeating_time != 0 and len(action_list)>1:
        random.seed(5)
        adjust_action_index = random.randint(0,len(action_list)-1)
    for i in range(len(action_list)):
        if action_list[i]=='Tip':
            try:
                if not i == len(action_list)-1:
                    rotation_pole, rotation_direction, rotation_angle, new_vertex_position = calculate_parameter_tip(begin_object_vertex_position, list(contact_state_trajectory[i]), None, profile_environment)
                    if repeating_time != 0 and i == adjust_action_index:
                        rotation_angle = rotation_angle + 10*random.randint(1,4)
                        if rotation_direction=='CCW':
                            new_vertex_position = [point_position_after_rotation(vertex, rotation_pole, rotation_angle) for vertex in bgin_object_vertex_position]
                        else:
                            new_vertex_position = [point_position_after_rotation(vertex, rotation_pole, -rotation_angle) for vertex in begin_object_vertex_position]
                        if obtain_contact_state(new_vertex_position, profile_environment) != list(contact_state_trajectory[i]):
                            return 'Infeasible'
                elif final_object_vertex_position != None:
                    rotation_pole, rotation_direction, rotation_angle, new_vertex_position = calculate_parameter_tip(begin_object_vertex_position, list(contact_state_trajectory[i]), final_object_vertex_position, profile_environment)
                else:
                    rotation_pole, rotation_direction, rotation_angle, new_vertex_position = calculate_parameter_tip(begin_object_vertex_position, list(contact_state_trajectory[i]), None, profile_environment)
                action_parameter.append([rotation_pole, rotation_direction, rotation_angle])
                object_pose_trajectory.append(new_vertex_position)
                begin_object_vertex_position = new_vertex_position
            except:
                return 'Infeasible'
        elif action_list[i]=='Push':
            try:
                if i != len(action_list)-1:
                    push_direction, push_distance, new_vertex_position=calculate_parameter_push(begin_object_vertex_position, list(contact_state_trajectory[i]), None, profile_environment)
                    if repeating_time != 0 and i == adjust_action_index:
                        push_distance = push_distance + 0.2*random.randint(1,4)
                        new_vertex_position = [[vertex[0]+push_distance*push_direction[0], vertex[1]+push_distance*push_direction[1]] for vertex in begin_object_vertex_position]
                        if obtain_contact_state(new_vertex_position, profile_environment) != list(contact_state_trajectory[i]):
                            return 'Infeasible'
                elif final_object_vertex_position != None:
                    push_direction, push_distance, new_vertex_position=calculate_parameter_push(begin_object_vertex_position, list(contact_state_trajectory[i]), final_object_vertex_position, profile_environment)
                else:
                     push_direction, push_distance, new_vertex_position=calculate_parameter_push(begin_object_vertex_position, list(contact_state_trajectory[i]), None, profile_environment) 
                action_parameter.append([push_direction, push_distance])
                object_pose_trajectory.append(new_vertex_position)
                begin_object_vertex_position = new_vertex_position
            except:
                return 'Infeasible'
        elif action_list[i]=='Tilting-slide':
            try:
                if i != len(action_list)-1:
                    delta_angle, new_vertex_position = calculate_parameter_tilting_slide(begin_object_vertex_position, list(contact_state_trajectory[i]), None, profile_environment)
                    if repeating_time != 0 and i == adjust_action_index:
                        if delta_angle>0:
                            random.seed(5)
                            delta_angle = delta_angle + 10*random.randint(1,4)
                        else:
                            random.seed(5)
                            delta_angle = delta_angle - 10*random.randint(1,4)
                        contact_edge_point=Tilting_slide_contact_edge_point(begin_object_vertex_position, list(contact_state_trajectory[i]), profile_environment)
                        new_vertex_position = Tilting_slide_calculate_next_vertex_position_simplified(contact_edge_point, delta_angle, begin_object_vertex_position, profile_environment)
                        if obtain_contact_state(new_vertex_position, profile_environment) != list(contact_state_trajectory[i]):
                            return 'Infeasible'
                elif final_object_vertex_position != None:
                    delta_angle, new_vertex_position = calculate_parameter_tilting_slide(begin_object_vertex_position, list(contact_state_trajectory[i]), final_object_vertex_position, profile_environment)
                else:
                    delta_angle, new_vertex_position = calculate_parameter_tilting_slide(begin_object_vertex_position, list(contact_state_trajectory[i]), None, profile_environment)
                action_parameter.append([list(contact_state_trajectory[i]), delta_angle])
                object_pose_trajectory.append(new_vertex_position)
                begin_object_vertex_position = new_vertex_position
            except:
                return 'Infeasible'
    return action_parameter, object_pose_trajectory

def object_pose_plan_within_contact_state(initial_object_vertex_position, final_object_vertex_position, profile_environment, accuracy = 0.004):
    initial_contact_state = obtain_contact_state(initial_object_vertex_position, profile_environment)
    final_contact_state = obtain_contact_state(final_object_vertex_position, profile_environment)
    if initial_contact_state != final_contact_state:
        return False
    for vertex in final_object_vertex_position:
        if Point(vertex).intersects(LineString(profile_environment).buffer(0.01))==True:
            final_pole = vertex
            break
    for vertex in initial_object_vertex_position:
        if Point(vertex).intersects(LineString(profile_environment).buffer(0.01))==True:
            pole = vertex
            break
    previous_action_total = []
    contact_state_trajectory = []
    action_parameter_trajectory = []
    object_pose_trajectory = []
    if 'single_contact' in initial_contact_state:
        delta_angle = calculate_intersect_angle_AB_CD(initial_object_vertex_position[0], initial_object_vertex_position[1], final_object_vertex_position[0], final_object_vertex_position[1])
        if delta_angle<0:
            rotate_direction = 'CW'
        else:
            rotate_direction = 'CCW'
        delta_distance = Point(pole).distance(Point(final_pole))
        if delta_distance>0.001:
            moving_direction_temp = [(final_pole[0]-pole[0])/delta_distance, (final_pole[1]-pole[1])/delta_distance]
            for j in range(len(profile_environment)-1):
                if Polygon(initial_object_vertex_position).buffer(accuracy).intersects(LineString([profile_environment[j], profile_environment[j+1]])) == True:
                    contact_env_edge=[profile_environment[j], profile_environment[j+1]]
                    break
            contact_env_edge_direction = [profile_environment[j+1][0]-profile_environment[j][0], profile_environment[j+1][1]-profile_environment[j][1]]
            contact_env_edge_direction = [contact_env_edge_direction[0]/LineString([profile_environment[j], profile_environment[j+1]]).length, contact_env_edge_direction[1]/LineString([profile_environment[j], profile_environment[j+1]]).length]
            if np.dot(contact_env_edge_direction, moving_direction_temp)>0:
                moving_direction = contact_env_edge_direction
            else:
                moving_direction = [-contact_env_edge_direction[0], -contact_env_edge_direction[1]]
        if abs(delta_angle)>1:
            previous_action_total.append('Tip')
            contact_state_trajectory.append(initial_contact_state)
            action_parameter_trajectory.append([pole, rotate_direction, abs(delta_angle)])
            object_pose_trajectory.append(Tip_next_vertex_position(initial_object_vertex_position, [pole, rotate_direction, abs(delta_angle)], profile_environment))
        if delta_distance>0.001:
            previous_action_total.append('Push')
            contact_state_trajectory.append(initial_contact_state)
            action_parameter_trajectory.append([moving_direction, delta_distance])
            object_pose_trajectory.append(final_object_vertex_position)
    elif 'double_contact' in initial_contact_state:
        delta_angle = calculate_intersect_angle_AB_CD(initial_object_vertex_position[0], initial_object_vertex_position[1], final_object_vertex_position[0], final_object_vertex_position[1])
        previous_action_total.append('Tilting-slide')
        contact_state_trajectory.append(initial_contact_state)
        action_parameter_trajectory.append([initial_contact_state, delta_angle])
        object_pose_trajectory.append(final_object_vertex_position)
    return previous_action_total, contact_state_trajectory, action_parameter_trajectory, object_pose_trajectory
