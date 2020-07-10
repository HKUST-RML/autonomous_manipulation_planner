#!/usr/bin/env python
import time
import re
import numpy as np
from general_functions import *
from top_and_middle_layer import *
from math import *
from jerk import *
from shapely.geometry import LineString
from shapely.geometry import Polygon
from shapely.geometry import Point
from shapely.geometry import LinearRing

def Feasible_Grasp(current_vertex_position, F1_position, F2_position, profile_environment, accuracy = 0.002):
    if F1_position != None and F2_position != None:
        if Point(F1_position).distance(Point(F2_position))<0.05:
            return False
        if Point(F1_position).distance(Point(F2_position))>1.4:
            return False
    if F1_position != None and LineString(profile_environment).distance(Point(F1_position))<accuracy:
        return False
    if F2_position != None and LineString(profile_environment).distance(Point(F2_position))<accuracy:
        return False
    current_vertex_position_contract0 = Polygon(current_vertex_position).buffer(-accuracy)
    if F1_position != None and current_vertex_position_contract0.intersects(Point(F1_position))==True:
        return False
    if F2_position != None and current_vertex_position_contract0.intersects(Point(F2_position))==True:
        return False
    for i in range(len(current_vertex_position)):
        if F1_position != None and Point(F1_position).distance(Point(current_vertex_position[i])) <0.1:
            return False
        if F2_position != None and Point(F2_position).distance(Point(current_vertex_position[i])) <0.1:
            return False
    if F1_position != None and F2_position != None:
        F1_F2_vector = [(F2_position[0]-F1_position[0])/LineString([F1_position, F2_position]).length, (F2_position[1]-F1_position[1])/LineString([F1_position, F2_position]).length]
        F2_F1_vector = [-F1_F2_vector[0], -F1_F2_vector[1]]
        F1_F2_perp1 = [-F1_F2_vector[1], F1_F2_vector[0]]
        F1_F2_perp2 = [F1_F2_vector[1], -F1_F2_vector[0]]
        F1_F1_F2_perp1 = LineString([F1_position, [F1_position[0]+2*F1_F2_perp1[0], F1_position[1]+2*F1_F2_perp1[1]]])
        F2_F1_F2_perp1 = LineString([F2_position, [F2_position[0]+2*F1_F2_perp1[0], F2_position[1]+2*F1_F2_perp1[1]]])
        F1_perp1_out = LineString([F1_position, [F1_position[0]+0.8*F1_F2_perp1[0]+0.35*F2_F1_vector[0], F1_position[1]+0.8*F1_F2_perp1[1]+0.35*F2_F1_vector[1]]])
        F2_perp1_out = LineString([F2_position, [F2_position[0]+0.8*F1_F2_perp1[0]+0.35*F1_F2_vector[0], F2_position[1]+0.8*F1_F2_perp1[1]+0.35*F1_F2_vector[1]]])
        F1_F1_F2_perp2 = LineString([F1_position, [F1_position[0]+2*F1_F2_perp2[0], F1_position[1]+2*F1_F2_perp2[1]]])
        F2_F1_F2_perp2 = LineString([F2_position, [F2_position[0]+2*F1_F2_perp2[0], F2_position[1]+2*F1_F2_perp2[1]]])
        F1_perp2_out = LineString([F1_position, [F1_position[0]+0.8*F1_F2_perp2[0]+0.35*F2_F1_vector[0], F1_position[1]+0.8*F1_F2_perp2[1]+0.35*F2_F1_vector[1]]])
        F2_perp2_out = LineString([F2_position, [F2_position[0]+0.8*F1_F2_perp2[0]+0.35*F1_F2_vector[0], F2_position[1]+0.8*F1_F2_perp2[1]+0.35*F1_F2_vector[1]]])
        current_vertex_position_contract = Polygon(current_vertex_position).buffer(-0.01)
        if (F1_F1_F2_perp1.distance(current_vertex_position_contract)>0 and F2_F1_F2_perp1.distance(current_vertex_position_contract)>0 and F1_perp1_out.distance(current_vertex_position_contract)>0 and F2_perp1_out.distance(current_vertex_position_contract)>0 and F1_F1_F2_perp1.distance(LineString(profile_environment))>accuracy and F2_F1_F2_perp1.distance(LineString(profile_environment))>accuracy and F1_perp1_out.distance(LineString(profile_environment))>accuracy and F2_perp1_out.distance(LineString(profile_environment))>accuracy):
            return True
        elif (F1_F1_F2_perp2.distance(current_vertex_position_contract)>0 and F2_F1_F2_perp2.distance(current_vertex_position_contract)>0 and F1_perp2_out.distance(current_vertex_position_contract)>0 and F2_perp2_out.distance(current_vertex_position_contract)>0 and F1_F1_F2_perp2.distance(LineString(profile_environment))>accuracy and F2_F1_F2_perp2.distance(LineString(profile_environment))>accuracy and F1_perp2_out.distance(LineString(profile_environment))>accuracy and F2_perp2_out.distance(LineString(profile_environment))>accuracy):
            return True
        else:
            return False
    else:
        return True

def system_config_list_tip(current_vertex_position, current_F1, current_F2, pole, direction, rotate_angle, tip_step = 2):
    object_pose_list = []
    F1_list = []
    F2_list = []
    for angle in np.arange(0, rotate_angle, tip_step):
        if direction == 'CCW':
            object_pose = [point_position_after_rotation(vertex, pole, angle) for vertex in current_vertex_position]
            object_pose_list.append(object_pose)
            F1 = point_position_after_rotation(current_F1, pole, angle)
            F1_list.append(F1)
            F2 = point_position_after_rotation(current_F2, pole, angle)
            F2_list.append(F2)
        else:
            object_pose = [point_position_after_rotation(vertex, pole, -angle) for vertex in current_vertex_position]
            object_pose_list.append(object_pose)
            F1 = point_position_after_rotation(current_F1, pole, -angle)
            F1_list.append(F1)
            F2 = point_position_after_rotation(current_F2, pole, -angle)
            F2_list.append(F2)
    if direction == 'CCW':
        object_pose = [point_position_after_rotation(vertex, pole, rotate_angle) for vertex in current_vertex_position]
        object_pose_list.append(object_pose)
        F1 = point_position_after_rotation(current_F1, pole, rotate_angle)
        F1_list.append(F1)
        F2 = point_position_after_rotation(current_F2, pole, rotate_angle)
        F2_list.append(F2)
    else:
        object_pose = [point_position_after_rotation(vertex, pole, -rotate_angle) for vertex in current_vertex_position]
        object_pose_list.append(object_pose)
        F1 = point_position_after_rotation(current_F1, pole, -rotate_angle)
        F1_list.append(F1)
        F2 = point_position_after_rotation(current_F2, pole, -rotate_angle)
        F2_list.append(F2) 
    return object_pose_list, F1_list, F2_list

def system_config_list_push(current_vertex_position, current_F1, current_F2, direction, distance, translate_step = 0.1):
    object_pose_list = []
    F1_list = []
    F2_list = []
    for translate_step in np.arange(0, distance, (distance-0.1)/20):
        object_pose = [[vertex[0]+translate_step*direction[0], vertex[1]+translate_step*direction[1]] for vertex in current_vertex_position]
        object_pose_list.append(object_pose)
        F1 = [current_F1[0]+translate_step*direction[0], current_F1[1]+translate_step*direction[1]]
        F1_list.append(F1)
        F2 = [current_F2[0]+translate_step*direction[0], current_F2[1]+translate_step*direction[1]]
        F2_list.append(F2)
    object_pose = [[vertex[0]+distance*direction[0], vertex[1]+distance*direction[1]] for vertex in current_vertex_position]
    object_pose_list.append(object_pose)
    F1 = [current_F1[0]+distance*direction[0], current_F1[1]+distance*direction[1]]
    F1_list.append(F1)
    F2 = [current_F2[0]+distance*direction[0], current_F2[1]+distance*direction[1]]
    F2_list.append(F2) 
    return object_pose_list, F1_list, F2_list

def system_config_list_move_in_air(current_vertex_position, current_F1, current_F2, direction, distance, translate_step = 0.1):
    object_pose_list = []
    F1_list = []
    F2_list = []
    for translate_step in np.arange(0, distance, (distance-0.1)/20):
        object_pose = [[vertex[0]+translate_step*direction[0], vertex[1]+translate_step*direction[1]] for vertex in current_vertex_position]
        object_pose_list.append(object_pose)
        F1 = [current_F1[0]+translate_step*direction[0], current_F1[1]+translate_step*direction[1]]
        F1_list.append(F1)
        F2 = [current_F2[0]+translate_step*direction[0], current_F2[1]+translate_step*direction[1]]
        F2_list.append(F2)
    object_pose = [[vertex[0]+distance*direction[0], vertex[1]+distance*direction[1]] for vertex in current_vertex_position]
    object_pose_list.append(object_pose)
    F1 = [current_F1[0]+distance*direction[0], current_F1[1]+distance*direction[1]]
    F1_list.append(F1)
    F2 = [current_F2[0]+distance*direction[0], current_F2[1]+distance*direction[1]]
    F2_list.append(F2) 
    return object_pose_list, F1_list, F2_list

def system_config_list_tilting_slide(current_vertex_position, current_F1, current_F2, target_contact_state, total_angle, profile_environment = [[0,5], [0,0], [5,0]]):
    object_pose_list = []
    F1_list = []
    F2_list = []
    current_contact_state = obtain_contact_state(current_vertex_position, profile_environment)
    contact_edge_point=Tilting_slide_contact_edge_point(current_vertex_position, target_contact_state, profile_environment)
    if total_angle>0:
        rotate_step = 0.5
    else:
        rotate_step = -0.5
    for angle in np.append(np.arange(0, total_angle, rotate_step), total_angle):
        new_vertex_position = Tilting_slide_calculate_next_vertex_position_simplified(contact_edge_point, angle, current_vertex_position, profile_environment)
        object_pose_list.append(new_vertex_position)
        [new_F1, new_F2] = Tilting_slide_calculate_next_vertex_position_simplified(contact_edge_point, angle, [current_F1, current_F2], profile_environment)
        F1_list.append(new_F1)
        F2_list.append(new_F2)
    return object_pose_list, F1_list, F2_list

def system_config_list(current_vertex_position, current_F1, current_F2, target_action, action_parameter, profile_environment = [[0,5], [0,0], [5,0]]):
    if target_action == 'Tip':
        [rotation_pole, rotation_direction, rotation_angle] = action_parameter
        return system_config_list_tip(current_vertex_position, current_F1, current_F2, rotation_pole, rotation_direction, rotation_angle)
    elif target_action == 'Push':
        [push_direction, push_distance] = action_parameter
        return system_config_list_push(current_vertex_position, current_F1, current_F2, push_direction, push_distance)
    elif target_action == 'Tilting-slide':
        [target_contact_state, A_r]=action_parameter
        return system_config_list_tilting_slide(current_vertex_position, current_F1, current_F2, target_contact_state, A_r, profile_environment)
    elif target_action == 'Move-in-air':
        [move_direction, move_distance] = action_parameter
        return system_config_list_move_in_air(current_vertex_position, current_F1, current_F2, move_direction, move_distance, profile_environment)
    else:
        return 'Wrong input'

def SuitableGrasp_tip_instantaneous(current_vertex_position, current_F1, current_F2, ini_vertex_position, ini_CoM_position, pole, direction, profile_environment, accuracy = 0.002):
    contact_index, contact_list, contact_normal_list, contact_local_tangent_list = contact_points_and_normal(current_vertex_position, current_F1, current_F2, profile_environment)
    contact_index_E = []
    contact_list_E = []
    contact_normal_list_E = []
    for i in range(len(contact_index)):
        if re.match('E', contact_index[i])!=None:
            contact_index_E.append(contact_index[i])
            contact_list_E.append(contact_list[i])
            contact_normal_list_E.append(contact_normal_list[i])
    contact_mode = {'F1': 'R', 'F2': 'R'}
    for i in range(len(contact_index_E)):
        radius = [contact_list_E[i][0]-pole[0], contact_list[i][1]-pole[1]]
        velocity = [-radius[1], radius[0]]
        if np.cross(radius, velocity)<0 and direction=='CCW':
            velocity = [-velocity[0], -velocity[1]]
        elif np.cross(radius, velocity)>0 and direction=='CW':
            velocity = [-velocity[0], -velocity[1]]
        if abs(sqrt(velocity[0]**2+velocity[1]**2))<0.001:
            contact_mode[contact_index_E[i]] = 'R'
        elif len((np.array(contact_normal_list_E[i])).shape)==1 and abs(np.dot(velocity, contact_normal_list_E[i]))<0.00001:
            if np.cross(velocity, contact_normal_list_E[i])>0:
                contact_mode[contact_index_E[i]] = 'S-'
            else:
                contact_mode[contact_index_E[i]] = 'S+'
        elif len((np.array(contact_normal_list_E[i])).shape)>1:
            for j in contact_normal_list_E[i]:
                if abs(np.dot(velocity, j))<0.00001:
                    if np.cross(velocity, j)>0:
                        contact_mode[contact_index_E[i]] = 'S-'
                    else:
                        contact_mode[contact_index_E[i]] = 'S+'
            else:
                contact_mode[contact_index_E[i]] = 'B'
        else:
            contact_mode[contact_index_E[i]] = 'B'
    current_CoM = calculate_current_CoM(ini_vertex_position, current_vertex_position, ini_CoM_position)
    if whether_stable(contact_index, contact_list, contact_normal_list, contact_local_tangent_list, current_CoM, contact_mode)==False:
        return False
    return True

def Suitable_Grasp_tip(current_vertex_position, current_F1, current_F2, ini_vertex_position, ini_CoM_position, pole, direction, rotate_angle, profile_environment):
    object_pose_list, F1_list, F2_list = system_config_list_tip(current_vertex_position, current_F1, current_F2, pole, direction, rotate_angle)
    for i in range(len(object_pose_list)):
        if Feasible_Grasp(object_pose_list[i], F1_list[i], F2_list[i], profile_environment)==False:
            return False
        if SuitableGrasp_tip_instantaneous(object_pose_list[i], F1_list[i], F2_list[i], ini_vertex_position, ini_CoM_position, pole, direction, profile_environment)==False: 
            return False
    return True

def SuitableGrasp_push_instantaneous(current_vertex_position, current_F1, current_F2, ini_vertex_position, ini_CoM_position, direction, profile_environment):
    contact_index, contact_list, contact_normal_list, contact_local_tangent_list = contact_points_and_normal(current_vertex_position, current_F1, current_F2, profile_environment)
    contact_index_E = []
    contact_list_E = []
    contact_normal_list_E = []
    for i in range(len(contact_index)):
        if re.match('E', contact_index[i])!=None:
            contact_index_E.append(contact_index[i])
            contact_list_E.append(contact_list[i])
            contact_normal_list_E.append(contact_normal_list[i])
    contact_mode = {'F1': 'R', 'F2': 'R'}
    for i in range(len(contact_index_E)):
        velocity = direction
        if abs(sqrt(velocity[0]**2+velocity[1]**2))<0.00001:
            contact_mode[contact_index_E[i]] = 'R'
        elif len((np.array(contact_normal_list_E[i])).shape)==1 and abs(np.dot(velocity, contact_normal_list_E[i]))<0.00001:
            if np.cross(velocity, contact_normal_list_E[i])>0:
                contact_mode[contact_index_E[i]] = 'S-'
            else:
                contact_mode[contact_index_E[i]] = 'S+'
        elif len((np.array(contact_normal_list_E[i])).shape)>1:
            for j in contact_normal_list_E[i]:
                if abs(np.dot(velocity, j))<0.00001:
                    if np.cross(velocity, j)>0:
                        contact_mode[contact_index_E[i]] = 'S-'
                    else:
                        contact_mode[contact_index_E[i]] = 'S+'
        else:
            contact_mode[contact_index_E[i]] = 'B'
    current_CoM = calculate_current_CoM(ini_vertex_position, current_vertex_position, ini_CoM_position)
    if whether_stable(contact_index, contact_list, contact_normal_list, contact_local_tangent_list, current_CoM, contact_mode)==False:
        return False
    return True

def Suitable_Grasp_push(current_vertex_position, current_F1, current_F2, ini_vertex_position, ini_CoM_position, direction, distance, profile_environment):
    object_pose_list, F1_list, F2_list = system_config_list_push(current_vertex_position, current_F1, current_F2, direction, distance)
    if abs(np.cross([(current_F2[0]-current_F1[0])/LineString([current_F1, current_F2]).length, (current_F2[1]-current_F1[1])/LineString([current_F1, current_F2]).length], direction))<0.001:
        return False
    for i in np.arange(0,len(object_pose_list)-2,len(object_pose_list)-3):
        if Feasible_Grasp(object_pose_list[i], F1_list[i], F2_list[i], profile_environment)==False:
            return False
        if SuitableGrasp_push_instantaneous(object_pose_list[i], F1_list[i], F2_list[i], ini_vertex_position, ini_CoM_position, direction, profile_environment)==False:
            return False
    return True

def SuitableGrasp_move_in_air_instantaneous(current_vertex_position, current_F1, current_F2, ini_vertex_position, ini_CoM_position, direction, profile_environment):
    contact_index, contact_list, contact_normal_list, contact_local_tangent_list = contact_points_and_normal(current_vertex_position, current_F1, current_F2, profile_environment)
    contact_index_E = []
    contact_list_E = []
    contact_normal_list_E = []
    for i in range(len(contact_index)):
        if re.match('E', contact_index[i])!=None:
            contact_index_E.append(contact_index[i])
            contact_list_E.append(contact_list[i])
            contact_normal_list_E.append(contact_normal_list[i])
    contact_mode = {'F1': 'R', 'F2': 'R'}
    for i in range(len(contact_index_E)):
        contact_mode[contact_index_E[i]] = 'B'
    current_CoM = calculate_current_CoM(ini_vertex_position, current_vertex_position, ini_CoM_position)
    if whether_stable(contact_index, contact_list, contact_normal_list, contact_local_tangent_list, current_CoM, contact_mode)==False:
        return False
    return True

def Suitable_Grasp_move_in_air(current_vertex_position, current_F1, current_F2, ini_vertex_position, ini_CoM_position, direction, distance, profile_environment):
    object_pose_list, F1_list, F2_list = system_config_list_push(current_vertex_position, current_F1, current_F2, direction, distance)
    for i in np.arange(0,len(object_pose_list)-2,len(object_pose_list)-3):
        if Feasible_Grasp(object_pose_list[i], F1_list[i], F2_list[i], profile_environment)==False:
            return False
        if SuitableGrasp_move_in_air_instantaneous(object_pose_list[i], F1_list[i], F2_list[i], ini_vertex_position, ini_CoM_position, direction, profile_environment)==False:
            return False
    return True

def SuitableGrasp_tilting_slide_instantaneous(current_vertex_position, current_F1, current_F2, ini_vertex_position, ini_CoM_position, target_contact_state, rotate_angle_direction, profile_environment = [[0,5], [0,0], [5,0]]):
    contact_index, contact_list, contact_normal_list, contact_local_tangent_list = contact_points_and_normal(current_vertex_position, current_F1, current_F2, profile_environment)
    contact_index_E = []
    contact_list_E = []
    contact_normal_list_E = []
    for i in range(len(contact_index)):
        if re.match('E', contact_index[i])!=None:
            contact_index_E.append(contact_index[i])
            contact_list_E.append(contact_list[i])
            contact_normal_list_E.append(contact_normal_list[i])
    contact_mode = {'F1': 'R', 'F2': 'R'}   
    contact_edge_point=Tilting_slide_contact_edge_point(current_vertex_position, target_contact_state, profile_environment) 
    for i in range(len(contact_index_E)):
        new_point = Tilting_slide_calculate_next_vertex_position_simplified(contact_edge_point, rotate_angle_direction, [contact_list_E[i]], profile_environment)[0]
        velocity = [new_point[0]-contact_list_E[i][0], new_point[1]-contact_list_E[i][1]]
        if abs(sqrt(velocity[0]**2+velocity[1]**2))<0.00001:
            contact_mode[contact_index_E[i]] = 'R'
        elif len((np.array(contact_normal_list_E[i])).shape)==1 and abs(np.dot(velocity, contact_normal_list_E[i]))<0.00001:
            if np.cross(velocity, contact_normal_list_E[i])>0:
                contact_mode[contact_index_E[i]] = 'S-'
            else:
                contact_mode[contact_index_E[i]] = 'S+'
        elif len((np.array(contact_normal_list_E[i])).shape)>1:
            for j in contact_normal_list_E[i]:
                if abs(np.dot(velocity, j))<0.00001:
                    if np.cross(velocity, j)>0:
                        contact_mode[contact_index_E[i]] = 'S-'
                    else:
                        contact_mode[contact_index_E[i]] = 'S+'
        else:
            contact_mode[contact_index_E[i]] = 'B'
    current_CoM = calculate_current_CoM(ini_vertex_position, current_vertex_position, ini_CoM_position)
    if whether_stable(contact_index, contact_list, contact_normal_list, contact_local_tangent_list, current_CoM, contact_mode)==False:
        return False
    return True

def Suitable_Grasp_tilting_slide(current_vertex_position, current_F1, current_F2, ini_vertex_position, ini_CoM_position, target_contact_state, A_r, profile_environment):
    object_pose_list, F1_list, F2_list = system_config_list_tilting_slide(current_vertex_position, current_F1, current_F2, target_contact_state, A_r, profile_environment)
    if A_r > 0:
        rotate_angle_direction = 1
    else:
        rotate_angle_direction = -1
    for i in np.arange(1,len(object_pose_list)-2,8):
        if Feasible_Grasp(object_pose_list[i], F1_list[i], F2_list[i], profile_environment)==False:
            return False
        if SuitableGrasp_tilting_slide_instantaneous(object_pose_list[i], F1_list[i], F2_list[i], ini_vertex_position, ini_CoM_position, target_contact_state, rotate_angle_direction, profile_environment)==False:
            return False
    return True

def Suitable_Grasp(current_vertex_position, F1_position, F2_position, ini_vertex_position, ini_CoM_position, target_action, action_parameter, profile_environment):
    if target_action == 'Tip':
        [rotation_pole, rotation_direction, rotation_angle] = action_parameter
        if Suitable_Grasp_tip(current_vertex_position, F1_position, F2_position, ini_vertex_position, ini_CoM_position, rotation_pole, rotation_direction, rotation_angle, profile_environment)==True:
            return True
        else:
            return False
    elif target_action == 'Push':
        [push_direction, push_distance] = action_parameter
        if Suitable_Grasp_push(current_vertex_position, F1_position, F2_position, ini_vertex_position, ini_CoM_position, push_direction, push_distance, profile_environment)==True:
            return True
        else:
            return False
    elif target_action == 'Tilting-slide':
        [target_contact_state, A_r]=action_parameter
        if Suitable_Grasp_tilting_slide(current_vertex_position, F1_position, F2_position, ini_vertex_position, ini_CoM_position, target_contact_state, A_r, profile_environment)==True:
            return True
        else:
            return False
    elif target_action == 'Move-in-air':
        [move_direction, move_distance] = action_parameter
        if Suitable_Grasp_move_in_air(current_vertex_position, F1_position, F2_position, ini_vertex_position, ini_CoM_position, move_direction, move_distance, profile_environment)==True:
            return True
        else:
            return False
    else:
        return False

def AdjustGraspAction_feasible_list(contact_index, contact_list, contact_normal_list, contact_local_tangent_list, current_CoM, current_F1_position, current_F2_position):
    feasible_action_list = []
    for action in ['Open','PivotF1CW','PivotF1CCW','PivotF2CW','PivotF2CCW','Slide+PivotF1CW', 'Slide+PivotF1CCW', 'Slide-PivotF1CW', 'Slide-PivotF1CCW','Slide+PivotF2CW', 'Slide+PivotF2CCW', 'Slide-PivotF2CW', 'Slide-PivotF2CCW', 'Slide+PivotF1CWClose', 'Slide+PivotF1CCWClose', 'Slide-PivotF1CWClose', 'Slide-PivotF1CCWClose','Slide+PivotF2CWClose', 'Slide+PivotF2CCWClose', 'Slide-PivotF2CWClose', 'Slide-PivotF2CCWClose']:
        if whether_AdjustGraspAction_feasible(contact_index, contact_list, contact_normal_list, contact_local_tangent_list, current_CoM, action, current_F1_position, current_F2_position)==True:
            feasible_action_list.append(action)
    return feasible_action_list

def whether_AdjustGraspAction_feasible(contact_index, contact_list, contact_normal_list, contact_local_tangent_list, current_CoM, action, current_F1_position, current_F2_position):
    temp_contact_index = contact_index[:]
    if 'F1' in temp_contact_index and Point(contact_list[temp_contact_index.index('F1')]).distance(Point(current_F2_position))<0.01:
        contact_index[temp_contact_index.index('F1')]='F2'
    if 'F2' in temp_contact_index and Point(contact_list[temp_contact_index.index('F2')]).distance(Point(current_F1_position))<0.01:
        contact_index[temp_contact_index.index('F2')]='F1'
    contact_index_E = []
    for i in range(len(contact_index)):
        if re.match('E', contact_index[i])!=None:
            contact_index_E.append(contact_index[i])
    contact_finger_number = len(contact_index)-len(contact_index_E)
    if contact_finger_number==0:
        if action == 'Open' or action == 'PivotF1CW' or action == 'PivotF1CCW' or action == 'PivotF2CW' or action == 'PivotF2CCW' or action == 'Slide+PivotF1CW' or action == 'Slide+PivotF1CCW' or action == 'Slide-PivotF1CW' or action == 'Slide-PivotF1CCW' or action == 'Slide+PivotF2CW' or action == 'Slide+PivotF2CCW' or action == 'Slide-PivotF2CW' or action == 'Slide-PivotF2CCW' or action == 'Slide+PivotF1CWClose' or action == 'Slide+PivotF1CCWClose' or action == 'Slide-PivotF1CWClose' or action == 'Slide-PivotF1CCWClose' or action == 'Slide+PivotF2CWClose' or action == 'Slide+PivotF2CCWClose' or action == 'Slide-PivotF2CWClose' or action == 'Slide-PivotF2CCWClose':
            return True
    elif contact_finger_number==1:
        if 'F1' in contact_index:
            if action == 'Open' or action == 'Close' or action == 'TranslateLeft' or action == 'TranslateRight' or action == 'TranslateUp' or action == 'TranslateDown':
                contact_mode0 = dict({'F1':'B'}, **dict.fromkeys(contact_index_E, 'R'))
                if whether_stable(contact_index, contact_list, contact_normal_list, contact_local_tangent_list, current_CoM, contact_mode0)==True:
                    return True
            elif action == 'PivotF1CW' or action == 'PivotF1CCW':
                contact_mode1 = dict({'F1':'R'}, **dict.fromkeys(contact_index_E, 'R'))
                if whether_stable(contact_index, contact_list, contact_normal_list, contact_local_tangent_list, current_CoM, contact_mode1)==True:
                    return True
            elif action == 'Slide+PivotF1CW' or action == 'Slide+PivotF1CCW' or action == 'Slide+PivotF1CWClose' or action == 'Slide+PivotF1CCWClose':
                contact_mode2 = dict({'F1':'S+'}, **dict.fromkeys(contact_index_E, 'R'))
                if whether_stable(contact_index, contact_list, contact_normal_list, contact_local_tangent_list, current_CoM, contact_mode2)==True:
                    return True
            elif action == 'Slide-PivotF1CW' or action == 'Slide-PivotF1CCW' or action == 'Slide-PivotF1CWClose' or action == 'Slide-PivotF1CCWClose':
                contact_mode3 = dict({'F1':'S-'}, **dict.fromkeys(contact_index_E, 'R'))
                if whether_stable(contact_index, contact_list, contact_normal_list, contact_local_tangent_list, current_CoM, contact_mode3)==True:
                    return True
        elif 'F2' in contact_index:
            if action == 'Open' or action == 'Close' or action == 'TranslateLeft' or action == 'TranslateRight' or action == 'TranslateUp' or action == 'TranslateDown':
                contact_mode0 = dict({'F2':'B'}, **dict.fromkeys(contact_index_E, 'R'))
                if whether_stable(contact_index, contact_list, contact_normal_list, contact_local_tangent_list, current_CoM, contact_mode0)==True:
                    return True
            elif action == 'PivotF2CW' or action == 'PivotF2CCW':
                contact_mode1 = dict({'F2':'R'}, **dict.fromkeys(contact_index_E, 'R'))
                if whether_stable(contact_index, contact_list, contact_normal_list, contact_local_tangent_list, current_CoM, contact_mode1)==True:
                    return True
            elif action == 'Slide+PivotF2CW' or action == 'Slide+PivotF2CCW' or action == 'Slide+PivotF2CWClose' or action == 'Slide+PivotF2CCWClose':
                contact_mode2 = dict({'F2':'S+'}, **dict.fromkeys(contact_index_E, 'R'))
                if whether_stable(contact_index, contact_list, contact_normal_list, contact_local_tangent_list, current_CoM, contact_mode2)==True:
                    return True
            elif action == 'Slide-PivotF2CW' or action == 'Slide-PivotF2CCW' or action == 'Slide-PivotF2CWClose' or action == 'Slide-PivotF2CCWClose':
                contact_mode3 = dict({'F2':'S-'}, **dict.fromkeys(contact_index_E, 'R'))
                if whether_stable(contact_index, contact_list, contact_normal_list, contact_local_tangent_list, current_CoM, contact_mode3)==True:
                    return True
    elif contact_finger_number==2:
        if action == 'Open' or action == 'Close' or action == 'TranslateLeft' or action == 'TranslateRight' or action == 'TranslateUp' or action == 'TranslateDown':
            contact_mode0 = dict({'F1':'B', 'F2': 'B'}, **dict.fromkeys(contact_index_E, 'R'))
            if whether_stable(contact_index, contact_list, contact_normal_list, contact_local_tangent_list, current_CoM, contact_mode0)==True:
                return True
        if action == 'PivotF1CW' or action == 'PivotF1CCW':
            contact_mode1 = dict({'F1':'R', 'F2': 'B'}, **dict.fromkeys(contact_index_E, 'R'))
            if whether_stable(contact_index, contact_list, contact_normal_list, contact_local_tangent_list, current_CoM, contact_mode1)==True:
                return True
        if action == 'PivotF2CW' or action == 'PivotF2CCW':
            contact_mode2 = dict({'F1':'B', 'F2': 'R'}, **dict.fromkeys(contact_index_E, 'R'))
            if whether_stable(contact_index, contact_list, contact_normal_list, contact_local_tangent_list, current_CoM, contact_mode2)==True:
                return True
        if action == 'Slide+PivotF1CW' or action == 'Slide+PivotF1CCW' or action == 'Slide+PivotF1CWClose' or action == 'Slide+PivotF1CCWClose' or action == 'Slide-PivotF1CW' or action == 'Slide-PivotF1CCW' or action == 'Slide-PivotF1CWClose' or action == 'Slide-PivotF1CCWClose' or action == 'Slide+PivotF2CW' or action == 'Slide+PivotF2CCW' or action == 'Slide+PivotF2CWClose' or action == 'Slide+PivotF2CCWClose' or action == 'Slide-PivotF2CW' or action == 'Slide-PivotF2CCW' or action == 'Slide-PivotF2CWClose' or action == 'Slide-PivotF2CCWClose':
            F1_contact_normal = contact_normal_list[contact_index.index('F1')]
            F2_contact_normal = contact_normal_list[contact_index.index('F2')]
            if action == 'Slide+PivotF1CW' or action == 'Slide+PivotF1CCW' or action == 'Slide+PivotF1CWClose' or action == 'Slide+PivotF1CCWClose':
                if contact_normal_list[contact_index.index('F1')][1]>0:
                    contact_mode3 = dict({'F1':'S+', 'F2': 'B'}, **dict.fromkeys(contact_index_E, 'R'))
                    if whether_stable(contact_index, contact_list, contact_normal_list, contact_local_tangent_list, current_CoM, contact_mode3)==True:
                        return True
            elif action == 'Slide-PivotF1CW' or action == 'Slide-PivotF1CCW' or action == 'Slide-PivotF1CWClose' or action == 'Slide-PivotF1CCWClose':
                if contact_normal_list[contact_index.index('F1')][1]>0:
                    contact_mode3 = dict({'F1':'S-', 'F2': 'B'}, **dict.fromkeys(contact_index_E, 'R'))
                    if whether_stable(contact_index, contact_list, contact_normal_list, contact_local_tangent_list, current_CoM, contact_mode3)==True:
                        return True
            elif action == 'Slide+PivotF2CW' or action == 'Slide+PivotF2CCW' or action == 'Slide+PivotF2CWClose' or action == 'Slide+PivotF2CCWClose':
                if contact_normal_list[contact_index.index('F2')][1]>0:
                    contact_mode3 = dict({'F1':'B', 'F2': 'S+'}, **dict.fromkeys(contact_index_E, 'R'))
                    if whether_stable(contact_index, contact_list, contact_normal_list, contact_local_tangent_list, current_CoM, contact_mode3)==True:
                        return True
            elif action == 'Slide-PivotF2CW' or action == 'Slide-PivotF2CCW' or action == 'Slide-PivotF2CWClose' or action == 'Slide-PivotF2CCWClose':
                if contact_normal_list[contact_index.index('F2')][1]>0:
                    contact_mode3 = dict({'F1':'B', 'F2': 'S-'}, **dict.fromkeys(contact_index_E, 'R'))
                    if whether_stable(contact_index, contact_list, contact_normal_list, contact_local_tangent_list, current_CoM, contact_mode3)==True:
                        return True
    return False

def finger_position_after_action(current_vertex_position, current_F1, current_F2, action, profile_environment, ini_vertex_position, ini_CoM_position, accuracy=0.01):
    target_position_set=[]
    action_feasible=True
    current_vertex_position_contract = Polygon(current_vertex_position).buffer(-0.000001)
    if action == 'Open':
        gripper_direction = [(current_F2[0]-current_F1[0])/LineString([current_F1, current_F2]).length, (current_F2[1]-current_F1[1])/LineString([current_F1, current_F2]).length]
        target_F1 = [current_F1[0]-0.05*gripper_direction[0], current_F1[1]-0.05*gripper_direction[1]]
        target_F2 = [current_F2[0]+0.05*gripper_direction[0], current_F2[1]+0.05*gripper_direction[1]]
        cost = 0.1
        if LineString([current_F1, current_F2]).length>1.3:
            action_feasible=False
        if action_feasible==True:
            target_position_set.append([target_F1, target_F2, cost])        
    elif action == 'PivotF1CCW':
        target_F1 = current_F1
        for angle in range(1,360):
            target_F2 = point_position_after_rotation(current_F2, current_F1, angle)
            if Point(target_F2).distance(LineString(profile_environment))<0.1:
                action_feasible=False
                break
            if angle<5 and current_vertex_position_contract.distance(Point(target_F2))<=0:
                action_feasible=False
                break
            elif angle>4 and Point(target_F2).distance(Polygon(current_vertex_position))<0.005:
                cost = LineString([current_F1, current_F2]).length*angle*pi/180
                break
        if action_feasible==True:
            target_position_set.append([target_F1, target_F2, cost]) 
    elif action == 'PivotF1CW':
        target_F1 = current_F1
        for angle in range(1,360):
            target_F2 = point_position_after_rotation(current_F2, current_F1, -angle)
            if Point(target_F2).distance(LineString(profile_environment))<0.1:
                action_feasible=False
                break
            if angle<5 and current_vertex_position_contract.distance(Point(target_F2))<=0:
                action_feasible=False
                break
            elif angle>4 and Point(target_F2).distance(Polygon(current_vertex_position))<0.005:
                cost = LineString([current_F1, current_F2]).length*angle*pi/180
                break
        if action_feasible==True:
            target_position_set.append([target_F1, target_F2, cost]) 
    elif action == 'PivotF2CCW':
        target_F2 = current_F2
        for angle in range(1,360):
            target_F1 = point_position_after_rotation(current_F1, current_F2, angle)
            if Point(target_F1).distance(LineString(profile_environment))<0.1:
                action_feasible=False
                break
            if angle<5 and current_vertex_position_contract.distance(Point(target_F1))<=0:
                action_feasible=False
                break
            elif angle>4 and Point(target_F1).distance(Polygon(current_vertex_position))<0.005:
                cost = LineString([current_F1, current_F2]).length*angle*pi/180
                break
        if action_feasible==True:
            target_position_set.append([target_F1, target_F2, cost]) 
    elif action == 'PivotF2CW':
        target_F2 = current_F2
        for angle in range(1,360):
            target_F1 = point_position_after_rotation(current_F1, current_F2, -angle)
            if Point(target_F1).distance(LineString(profile_environment))<0.1:
                action_feasible=False
                break
            if angle<5 and current_vertex_position_contract.distance(Point(target_F1))<=0:
                action_feasible=False
                break
            elif angle>4 and Point(target_F1).distance(Polygon(current_vertex_position))<0.005:
                cost = LineString([current_F1, current_F2]).length*angle*pi/180
                break
        if action_feasible==True:
            target_position_set.append([target_F1, target_F2, cost]) 
    elif action == 'Slide+PivotF1CW' or action == 'Slide+PivotF1CCW' or action == 'Slide-PivotF1CW' or action == 'Slide-PivotF1CCW':
        min_distance = float('inf')
        for i in range(len(current_vertex_position)):
            if Point(current_F1).distance(LineString([current_vertex_position[i],current_vertex_position[(i+1)%(len(current_vertex_position))]]))<min_distance:
                contact_edge = [current_vertex_position[i], current_vertex_position[(i+1)%(len(current_vertex_position))]]
                min_distance = Point(current_F1).distance(LineString([current_vertex_position[i],current_vertex_position[(i+1)%(len(current_vertex_position))]]))
        current_F1 = footCtoAB(contact_edge[0],contact_edge[1],current_F1)
        contact_edge_length = LineString(contact_edge).length
        slide_direction = [(contact_edge[1][0]-contact_edge[0][0])/contact_edge_length, (contact_edge[1][1]-contact_edge[0][1])/contact_edge_length]
        geometric_center = (sum(np.array(current_vertex_position).astype(float))/len(current_vertex_position)).tolist()
        F1_to_center = [geometric_center[0]-current_F1[0], geometric_center[1]-current_F1[1]]
        if (action == 'Slide+PivotF1CW' or action == 'Slide+PivotF1CCW') and np.cross(F1_to_center, slide_direction)>0:
            slide_direction = [-slide_direction[0], -slide_direction[1]]
        elif (action == 'Slide-PivotF1CW' or action == 'Slide-PivotF1CCW') and np.cross(F1_to_center, slide_direction)<0:
            slide_direction = [-slide_direction[0], -slide_direction[1]]
        temp_contact_index, temp_contact_list, temp_contact_normal_list, temp_contact_local_tangent_list = contact_points_and_normal(current_vertex_position, current_F1, current_F2, profile_environment)
        temp0_contact_index = temp_contact_index[:]
        temp0_contact_list = temp_contact_list[:]
        temp0_contact_normal_list = temp_contact_normal_list[:]
        temp0_contact_local_tangent_list = temp_contact_local_tangent_list[:]
        for i in range(len(temp0_contact_index)):
            if temp0_contact_index[i] == 'F1' or temp0_contact_index[i] == 'F2':
                temp_contact_index.remove(temp0_contact_index[i])
                temp_contact_list.remove(temp0_contact_list[i])
                temp_contact_normal_list.remove(temp0_contact_normal_list[i])
                temp_contact_local_tangent_list.remove(temp0_contact_local_tangent_list[i])
        for sliding_length in np.arange(0.05, 0.8, 0.05):
            target_F1 = [current_F1[0]+sliding_length*slide_direction[0], current_F1[1]+sliding_length*slide_direction[1]]
            target_F2 = [current_F2[0]+sliding_length*slide_direction[0], current_F2[1]+sliding_length*slide_direction[1]]
            cost = sliding_length
            temp_contact_index_finger, temp_contact_list_finger, temp_contact_normal_list_finger, temp_contact_local_tangent_list_finger = contact_points_and_normal_finger(current_vertex_position, target_F1, target_F2, profile_environment)
            contact_index = temp_contact_index+temp_contact_index_finger
            contact_list = temp_contact_list+temp_contact_list_finger
            contact_normal_list = temp_contact_normal_list + temp_contact_normal_list_finger
            contact_local_tangent_list = temp_contact_local_tangent_list + temp_contact_local_tangent_list_finger
            current_CoM = calculate_current_CoM(ini_vertex_position, current_vertex_position, ini_CoM_position)
            if whether_AdjustGraspAction_feasible(contact_index, contact_list, contact_normal_list, contact_local_tangent_list, current_CoM, action, target_F1, target_F2)==False:
                break
            if Feasible_Grasp(current_vertex_position, target_F1, target_F2, profile_environment)==False:
                break
            temp_target_F1 = target_F1
            temp_target_F2 = target_F2
            if action == 'Slide+PivotF1CCW' or action == 'Slide-PivotF1CCW':
                for angle in range(1,360):
                    target_F2 = point_position_after_rotation(temp_target_F2, temp_target_F1, angle)
                    if Point(target_F2).distance(LineString(profile_environment))<0.1:
                        action_feasible=False
                        break
                    if  angle<5 and current_vertex_position_contract.distance(Point(target_F2))<=0:
                        action_feasible=True
                        target_F2 = temp_target_F2
                        break
                    elif angle>4 and Point(target_F2).distance(Polygon(current_vertex_position))<0.005:
                        cost = sliding_length+LineString([current_F1, current_F2]).length*angle*pi/180 
                        break
            elif action == 'Slide+PivotF1CW' or action == 'Slide-PivotF1CW':
                for angle in range(1,360):
                    target_F2 = point_position_after_rotation(temp_target_F2, temp_target_F1, -angle)
                    if Point(target_F2).distance(LineString(profile_environment))<0.1:
                        action_feasible=False
                        break
                    if angle<5 and current_vertex_position_contract.distance(Point(target_F2))<=0:
                        action_feasible=True
                        target_F2 = temp_target_F2
                        break
                    elif angle>4 and Point(target_F2).distance(Polygon(current_vertex_position))<0.005:
                        cost = sliding_length+LineString([current_F1, current_F2]).length*angle*pi/180
                        break
            if action_feasible==True:
                target_position_set.append([target_F1, target_F2, cost])
    elif action == 'Slide+PivotF2CW' or action == 'Slide+PivotF2CCW' or action == 'Slide-PivotF2CW' or action == 'Slide-PivotF2CCW':
        min_distance = float('inf')
        for i in range(len(current_vertex_position)):
            if Point(current_F2).distance(LineString([current_vertex_position[i],current_vertex_position[(i+1)%(len(current_vertex_position))]]))<min_distance:
                contact_edge = [current_vertex_position[i], current_vertex_position[(i+1)%(len(current_vertex_position))]]
                min_distance = Point(current_F2).distance(LineString([current_vertex_position[i],current_vertex_position[(i+1)%(len(current_vertex_position))]]))
        current_F2 = footCtoAB(contact_edge[0],contact_edge[1],current_F2)
        contact_edge_length = LineString(contact_edge).length
        slide_direction = [(contact_edge[1][0]-contact_edge[0][0])/contact_edge_length, (contact_edge[1][1]-contact_edge[0][1])/contact_edge_length]
        geometric_center = (sum(np.array(current_vertex_position).astype(float))/len(current_vertex_position)).tolist()
        F2_to_center = [geometric_center[0]-current_F2[0], geometric_center[1]-current_F2[1]]
        if (action == 'Slide+PivotF2CW' or action == 'Slide+PivotF2CCW') and np.cross(F2_to_center, slide_direction)>0:
            slide_direction = [-slide_direction[0], -slide_direction[1]]
        elif (action == 'Slide-PivotF2CW' or action == 'Slide-PivotF2CCW') and np.cross(F2_to_center, slide_direction)<0:
            slide_direction = [-slide_direction[0], -slide_direction[1]]
        temp_contact_index, temp_contact_list, temp_contact_normal_list, temp_contact_local_tangent_list = contact_points_and_normal(current_vertex_position, current_F1, current_F2, profile_environment)
        temp0_contact_index = temp_contact_index[:]
        temp0_contact_list = temp_contact_list[:]
        temp0_contact_normal_list = temp_contact_normal_list[:]
        temp0_contact_local_tangent_list = temp_contact_local_tangent_list[:]
        for i in range(len(temp0_contact_index)):
            if temp0_contact_index[i] == 'F1' or temp0_contact_index[i] == 'F2':
                temp_contact_index.remove(temp0_contact_index[i])
                temp_contact_list.remove(temp0_contact_list[i])
                temp_contact_normal_list.remove(temp0_contact_normal_list[i])
                temp_contact_local_tangent_list.remove(temp0_contact_local_tangent_list[i])
        for sliding_length in np.arange(0.05, 0.8, 0.05):
            target_F1 = [current_F1[0]+sliding_length*slide_direction[0], current_F1[1]+sliding_length*slide_direction[1]]
            target_F2 = [current_F2[0]+sliding_length*slide_direction[0], current_F2[1]+sliding_length*slide_direction[1]]
            cost = sliding_length
            temp_contact_index_finger, temp_contact_list_finger, temp_contact_normal_list_finger, temp_contact_local_tangent_list_finger = contact_points_and_normal_finger(current_vertex_position, target_F1, target_F2, profile_environment)
            contact_index = temp_contact_index+temp_contact_index_finger
            contact_list = temp_contact_list+temp_contact_list_finger
            contact_normal_list = temp_contact_normal_list + temp_contact_normal_list_finger
            contact_local_tangent_list = temp_contact_local_tangent_list + temp_contact_local_tangent_list_finger
            current_CoM = calculate_current_CoM(ini_vertex_position, current_vertex_position, ini_CoM_position)
            if whether_AdjustGraspAction_feasible(contact_index, contact_list, contact_normal_list, contact_local_tangent_list, current_CoM, action, target_F1, target_F2)==False:
                break
            if Feasible_Grasp(current_vertex_position, target_F1, target_F2, profile_environment)==False:
                break
            temp_target_F1 = target_F1
            temp_target_F2 = target_F2
            if action == 'Slide+PivotF2CCW' or action == 'Slide-PivotF2CCW':
                for angle in range(1,360):
                    target_F1 = point_position_after_rotation(temp_target_F1, temp_target_F2, angle)
                    if Point(target_F1).distance(LineString(profile_environment))<0.1:
                        action_feasible=False
                        break
                    if angle<5 and current_vertex_position_contract.distance(Point(target_F1))<=0:
                        action_feasible=True
                        target_F1 = temp_target_F1
                        break
                    elif angle>4 and Point(target_F1).distance(Polygon(current_vertex_position))<0.005:
                        cost = sliding_length+LineString([current_F1, current_F2]).length*angle*pi/180 
                        break            
            elif action == 'Slide+PivotF2CW' or action == 'Slide-PivotF2CW':
                for angle in range(1,360):
                    target_F1 = point_position_after_rotation(temp_target_F1, temp_target_F2, -angle)
                    if Point(target_F1).distance(LineString(profile_environment))<0.1:
                        action_feasible=False
                        break
                    if angle<5 and current_vertex_position_contract.distance(Point(target_F1))<=0:
                        action_feasible=True
                        target_F1 = temp_target_F1
                        break
                    elif angle>4 and Point(target_F1).distance(Polygon(current_vertex_position))<0.005:
                        cost = sliding_length+LineString([current_F1, current_F2]).length*angle*pi/180 
                        break
            if action_feasible==True:
                target_position_set.append([target_F1, target_F2, cost])

    elif action == 'Slide+PivotF1CWClose' or action == 'Slide+PivotF1CCWClose' or action == 'Slide-PivotF1CWClose' or action == 'Slide-PivotF1CCWClose':
        min_distance = float('inf')
        for i in range(len(current_vertex_position)):
            if Point(current_F1).distance(LineString([current_vertex_position[i],current_vertex_position[(i+1)%(len(current_vertex_position))]]))<min_distance:
                contact_edge = [current_vertex_position[i], current_vertex_position[(i+1)%(len(current_vertex_position))]]
                min_distance = Point(current_F1).distance(LineString([current_vertex_position[i],current_vertex_position[(i+1)%(len(current_vertex_position))]]))
        current_F1 = footCtoAB(contact_edge[0],contact_edge[1],current_F1)
        contact_edge_length = LineString(contact_edge).length
        slide_direction = [(contact_edge[1][0]-contact_edge[0][0])/contact_edge_length, (contact_edge[1][1]-contact_edge[0][1])/contact_edge_length]
        geometric_center = (sum(np.array(current_vertex_position).astype(float))/len(current_vertex_position)).tolist()
        F1_to_center = [geometric_center[0]-current_F1[0], geometric_center[1]-current_F1[1]]
        if (action == 'Slide+PivotF1CWClose' or action == 'Slide+PivotF1CCWClose') and np.cross(F1_to_center, slide_direction)>0:
            slide_direction = [-slide_direction[0], -slide_direction[1]]
        elif (action == 'Slide-PivotF1CWClose' or action == 'Slide-PivotF1CCWClose') and np.cross(F1_to_center, slide_direction)<0:
            slide_direction = [-slide_direction[0], -slide_direction[1]]
        temp_contact_index, temp_contact_list, temp_contact_normal_list, temp_contact_local_tangent_list = contact_points_and_normal(current_vertex_position, current_F1, current_F2, profile_environment)
        temp0_contact_index = temp_contact_index[:]
        temp0_contact_list = temp_contact_list[:]
        temp0_contact_normal_list = temp_contact_normal_list[:]
        temp0_contact_local_tangent_list = temp_contact_local_tangent_list[:]
        for i in range(len(temp0_contact_index)):
            if temp0_contact_index[i] == 'F1' or temp0_contact_index[i] == 'F2':
                temp_contact_index.remove(temp0_contact_index[i])
                temp_contact_list.remove(temp0_contact_list[i])
                temp_contact_normal_list.remove(temp0_contact_normal_list[i])
                temp_contact_local_tangent_list.remove(temp0_contact_local_tangent_list[i])
        for sliding_length in np.arange(0.05, 0.6, 0.05):
            target_F1 = [current_F1[0]+sliding_length*slide_direction[0], current_F1[1]+sliding_length*slide_direction[1]]
            target_F2 = [current_F2[0]+sliding_length*slide_direction[0], current_F2[1]+sliding_length*slide_direction[1]]
            cost = sliding_length
            temp_contact_index_finger, temp_contact_list_finger, temp_contact_normal_list_finger, temp_contact_local_tangent_list_finger = contact_points_and_normal_finger(current_vertex_position, target_F1, target_F2, profile_environment)
            contact_index = temp_contact_index+temp_contact_index_finger
            contact_list = temp_contact_list+temp_contact_list_finger
            contact_normal_list = temp_contact_normal_list + temp_contact_normal_list_finger
            contact_local_tangent_list = temp_contact_local_tangent_list + temp_contact_local_tangent_list_finger
            current_CoM = calculate_current_CoM(ini_vertex_position, current_vertex_position, ini_CoM_position)
            if whether_AdjustGraspAction_feasible(contact_index, contact_list, contact_normal_list, contact_local_tangent_list, current_CoM, action, target_F1, target_F2)==False:
                break
            if Feasible_Grasp(current_vertex_position, target_F1, target_F2, profile_environment)==False:
                break
            temp_target_F1 = target_F1
            temp_target_F2 = target_F2
            for angle in range(1,360):
                if 'CCW' in action:
                    target_F2 = point_position_after_rotation(temp_target_F2, temp_target_F1, angle)
                else:
                    target_F2 = point_position_after_rotation(temp_target_F2, temp_target_F1, -angle)
                if Point(target_F2).distance(LineString(profile_environment))<0.1:
                    target_position_set = []
                    break
                if angle<5 and current_vertex_position_contract.distance(Point(target_F2))<=0:
                    target_position_set = []
                    break
                if angle>4 and Point(target_F2).distance(Polygon(current_vertex_position))<0.005:
                    break 
                target_F2_list = LineString([target_F1, target_F2]).intersection(LinearRing(current_vertex_position))
                try:
                    target_F2_list = [list(list(target_F2_list.coords)[i]) for i in range(len(list(target_F2_list.coords)))]
                except:
                    target_F2_list = [[list(target_F2_list[i].coords[j]) for j in range(len(target_F2_list[i].coords))][0] for i in range(len(target_F2_list))]
                for point in target_F2_list:
                    if Point(point).distance(Point(target_F1))>0.1:
                        cost = sliding_length+LineString([current_F1, current_F2]).length*angle*pi/180+Point(point).distance(Point(target_F2))
                        target_position_set.append([target_F1, point, cost])
    elif action == 'Slide+PivotF2CWClose' or action == 'Slide+PivotF2CCWClose' or action == 'Slide-PivotF2CWClose' or action == 'Slide-PivotF2CCWClose':
        min_distance = float('inf')
        for i in range(len(current_vertex_position)):
            if Point(current_F2).distance(LineString([current_vertex_position[i],current_vertex_position[(i+1)%(len(current_vertex_position))]]))<min_distance:
                contact_edge = [current_vertex_position[i], current_vertex_position[(i+1)%(len(current_vertex_position))]]
                min_distance = Point(current_F2).distance(LineString([current_vertex_position[i],current_vertex_position[(i+1)%(len(current_vertex_position))]]))
        current_F2 = footCtoAB(contact_edge[0],contact_edge[1],current_F2)
        contact_edge_length = LineString(contact_edge).length
        slide_direction = [(contact_edge[1][0]-contact_edge[0][0])/contact_edge_length, (contact_edge[1][1]-contact_edge[0][1])/contact_edge_length]
        geometric_center = (sum(np.array(current_vertex_position).astype(float))/len(current_vertex_position)).tolist()
        F2_to_center = [geometric_center[0]-current_F2[0], geometric_center[1]-current_F2[1]]
        if (action == 'Slide+PivotF2CW' or action == 'Slide+PivotF2CCW') and np.cross(F2_to_center, slide_direction)>0:
            slide_direction = [-slide_direction[0], -slide_direction[1]]
        elif (action == 'Slide-PivotF2CW' or action == 'Slide-PivotF2CCW') and np.cross(F2_to_center, slide_direction)<0:
            slide_direction = [-slide_direction[0], -slide_direction[1]]
        temp_contact_index, temp_contact_list, temp_contact_normal_list, temp_contact_local_tangent_list = contact_points_and_normal(current_vertex_position, current_F1, current_F2, profile_environment)
        temp0_contact_index = temp_contact_index[:]
        temp0_contact_list = temp_contact_list[:]
        temp0_contact_normal_list = temp_contact_normal_list[:]
        temp0_contact_local_tangent_list = temp_contact_local_tangent_list[:]
        for i in range(len(temp0_contact_index)):
            if temp0_contact_index[i] == 'F1' or temp0_contact_index[i] == 'F2':
                temp_contact_index.remove(temp0_contact_index[i])
                temp_contact_list.remove(temp0_contact_list[i])
                temp_contact_normal_list.remove(temp0_contact_normal_list[i])
                temp_contact_local_tangent_list.remove(temp0_contact_local_tangent_list[i])
        for sliding_length in np.arange(0.05, 0.6, 0.05):
            target_F1 = [current_F1[0]+sliding_length*slide_direction[0], current_F1[1]+sliding_length*slide_direction[1]]
            target_F2 = [current_F2[0]+sliding_length*slide_direction[0], current_F2[1]+sliding_length*slide_direction[1]]
            cost = sliding_length
            temp_contact_index_finger, temp_contact_list_finger, temp_contact_normal_list_finger, temp_contact_local_tangent_list_finger = contact_points_and_normal_finger(current_vertex_position, target_F1, target_F2, profile_environment)
            contact_index = temp_contact_index+temp_contact_index_finger
            contact_list = temp_contact_list+temp_contact_list_finger
            contact_normal_list = temp_contact_normal_list + temp_contact_normal_list_finger
            contact_local_tangent_list = temp_contact_local_tangent_list + temp_contact_local_tangent_list_finger
            current_CoM = calculate_current_CoM(ini_vertex_position, current_vertex_position, ini_CoM_position)
            if whether_AdjustGraspAction_feasible(contact_index, contact_list, contact_normal_list, contact_local_tangent_list, current_CoM, action, target_F1, target_F2)==False:
                break
            if Feasible_Grasp(current_vertex_position, target_F1, target_F2, profile_environment)==False:
                break
            temp_target_F1 = target_F1
            temp_target_F2 = target_F2
            for angle in range(1,360):
                if 'CCW' in action:
                    target_F1 = point_position_after_rotation(temp_target_F1, temp_target_F2, angle)
                else:
                    target_F1 = point_position_after_rotation(temp_target_F1, temp_target_F2, -angle)
                if Point(target_F1).distance(LineString(profile_environment))<0.1:
                    target_position_set = []
                    break
                if angle<5 and current_vertex_position_contract.distance(Point(target_F1))<=0:
                    target_position_set = []
                    break
                if angle>4 and Point(target_F1).distance(Polygon(current_vertex_position))<0.005:
                    break 
                target_F1_list = LineString([target_F1, target_F2]).intersection(LinearRing(current_vertex_position))
                try:
                    target_F1_list = [list(list(target_F1_list.coords)[i]) for i in range(len(list(target_F1_list.coords)))]
                except:
                    target_F1_list = [[list(target_F1_list[i].coords[j]) for j in range(len(target_F1_list[i].coords))][0] for i in range(len(target_F1_list))]
                for point in target_F1_list:
                    if Point(point).distance(Point(target_F2))>0.1:
                        cost = sliding_length+LineString([current_F1, current_F2]).length*angle*pi/180+Point(point).distance(Point(target_F1))
                        target_position_set.append([point, target_F2, cost])                       
    return target_position_set            

def from_real_position_to_index(current_vertex_position, finger_position):
    if Point(finger_position).distance(Polygon(current_vertex_position))<0.01:
        point_index_temp = finger_position2index(current_vertex_position, finger_position)
        circumference0 = circumference(current_vertex_position)
        return ceil(point_index_temp/(circumference0/100.0))
    else:
        return 0

def from_index_to_real_position(current_vertex_position, finger_index):
    if finger_index <= 0:
        return None
    else:
        point_index_temp = (finger_index-0.5)*(circumference(current_vertex_position)/100.0)
        point = finger_index2position(current_vertex_position, point_index_temp)
        return point

def action_trajectory(current_vertex_position, current_grasp_index, target_grasp_index, action):
    [current_F1, current_F2] = [from_index_to_real_position(current_vertex_position, current_grasp_index[0]), from_index_to_real_position(current_vertex_position, current_grasp_index[1])]
    if action == 'Open':
        gripper_direction = [(current_F2[0]-current_F1[0])/LineString([current_F1, current_F2]).length, (current_F2[1]-current_F1[1])/LineString([current_F1, current_F2]).length]
        target_F1 = [current_F1[0]-0.05*gripper_direction[0], current_F1[1]-0.05*gripper_direction[1]]
        target_F2 = [current_F2[0]+0.05*gripper_direction[0], current_F2[1]+0.05*gripper_direction[1]]
        return [[current_F1, current_F2], [target_F1, target_F2]]
    [target_F1, target_F2] = [from_index_to_real_position(current_vertex_position, target_grasp_index[0]), from_index_to_real_position(current_vertex_position, target_grasp_index[1])]
    intersect_angle = calculate_intersect_angle_AB_CD(current_F1, current_F2, target_F1, target_F2)
    if 'CCW' in action and intersect_angle<0:
        intersect_angle=360+intersect_angle
    elif 'CCW' not in action and intersect_angle>0:
        intersect_angle = -360+intersect_angle
    trajectory = []
    if action == 'PivotF1CCW' or action == 'PivotF1CW':
        try:
            for angle in np.arange(0, intersect_angle, intersect_angle/15):
                trajectory.append([target_F1, point_position_after_rotation(current_F2, current_F1, angle)])
        except:
            pass
        trajectory.append([target_F1, target_F2])
    elif action == 'PivotF2CCW' or action == 'PivotF2CW':
        try:
            for angle in np.arange(0, intersect_angle, intersect_angle/15):
                trajectory.append([point_position_after_rotation(current_F1, current_F2, angle), target_F2])
        except:
            pass
        trajectory.append([target_F1, target_F2])
    elif action == 'Slide+PivotF1CCW' or action == 'Slide+PivotF1CW' or action == 'Slide-PivotF1CCW' or action == 'Slide-PivotF1CW':
        translate_distance = [target_F1[0] - current_F1[0], target_F1[1] - current_F1[1]]
        for i in range(11):
            trajectory.append([[current_F1[0]+(translate_distance[0]/10.0)*i, current_F1[1]+(translate_distance[1]/10.0)*i], [current_F2[0]+(translate_distance[0]/10.0)*i, current_F2[1]+(translate_distance[1]/10.0)*i]])
        after_translate = trajectory[-1]
        try:
            revert_angle=False
            for angle in np.arange(0, intersect_angle, intersect_angle/15):
                trajectory.append([target_F1, point_position_after_rotation(after_translate[1], after_translate[0], angle)])
                if Point(point_position_after_rotation(after_translate[1], after_translate[0], angle)).intersects(Polygon(current_vertex_position).buffer(-0.01))==True:
                    revert_angle = True
                    break
            if revert_angle == True:
                if intersect_angle<0:
                    intersect_angle=360+intersect_angle
                else:
                    intersect_angle = -360+intersect_angle
        except:
            pass
        trajectory.append([target_F1, target_F2])
    elif action == 'Slide+PivotF2CCW' or action == 'Slide+PivotF2CW' or action == 'Slide-PivotF2CCW' or action == 'Slide-PivotF2CW':
        translate_distance = [target_F2[0] - current_F2[0], target_F2[1] - current_F2[1]]
        for i in range(11):
            trajectory.append([[current_F1[0]+(translate_distance[0]/10.0)*i, current_F1[1]+(translate_distance[1]/10.0)*i], [current_F2[0]+(translate_distance[0]/10.0)*i, current_F2[1]+(translate_distance[1]/10.0)*i]])
        after_translate = trajectory[-1]
        try:
            revert_angle=False
            for angle in np.arange(0, intersect_angle, intersect_angle/15):
                trajectory.append([point_position_after_rotation(after_translate[0], after_translate[1], angle), target_F2])
                if Point(point_position_after_rotation(after_translate[0], after_translate[1], angle)).intersects(Polygon(current_vertex_position).buffer(-0.01))==True:
                    revert_angle = True
                    break
            if revert_angle == True:
                if intersect_angle<0:
                    intersect_angle=360+intersect_angle
                else:
                    intersect_angle = -360+intersect_angle
        except:
            pass
        trajectory.append([target_F1, target_F2])
    elif action == 'Slide+PivotF1CCWClose' or action == 'Slide+PivotF1CWClose' or action == 'Slide-PivotF1CCWClose' or action == 'Slide-PivotF1CWClose':
        translate_distance = [target_F1[0] - current_F1[0], target_F1[1] - current_F1[1]]
        for i in range(11):
            trajectory.append([[current_F1[0]+(translate_distance[0]/10.0)*i, current_F1[1]+(translate_distance[1]/10.0)*i], [current_F2[0]+(translate_distance[0]/10.0)*i, current_F2[1]+(translate_distance[1]/10.0)*i]])
        after_translate = trajectory[-1]
        try:
            revert_angle=False
            for angle in np.arange(0, intersect_angle, intersect_angle/15):
                trajectory.append([target_F1, point_position_after_rotation(after_translate[1], after_translate[0], angle)])
                if Point(point_position_after_rotation(after_translate[1], after_translate[0], angle)).intersects(Polygon(current_vertex_position).buffer(-0.01))==True:
                    revert_angle = True
                    break
            if revert_angle == True:
                if intersect_angle<0:
                    intersect_angle=360+intersect_angle
                else:
                    intersect_angle = -360+intersect_angle
        except:
            pass
        Point_after_rotation = [target_F1, point_position_after_rotation(after_translate[1], after_translate[0], intersect_angle)]
        trajectory.append([target_F1, [(Point_after_rotation[1][0]+target_F2[0])/2, (Point_after_rotation[1][1]+target_F2[1])/2]])
        trajectory.append([target_F1, target_F2])
    elif action == 'Slide+PivotF2CCWClose' or action == 'Slide+PivotF2CWClose' or action == 'Slide-PivotF2CCWClose' or action == 'Slide-PivotF2CWClose':
        translate_distance = [target_F2[0] - current_F2[0], target_F2[1] - current_F2[1]]
        for i in range(11):
            trajectory.append([[current_F1[0]+(translate_distance[0]/10.0)*i, current_F1[1]+(translate_distance[1]/10.0)*i], [current_F2[0]+(translate_distance[0]/10.0)*i, current_F2[1]+(translate_distance[1]/10.0)*i]])
        after_translate = trajectory[-1]
        try:
            revert_angle=False
            for angle in np.arange(0, intersect_angle, intersect_angle/15):
                trajectory.append([point_position_after_rotation(after_translate[0], after_translate[1], angle), target_F2])
                if Point(point_position_after_rotation(after_translate[0], after_translate[1], angle)).intersects(Polygon(current_vertex_position).buffer(-0.01))==True:
                    revert_angle = True
                    break
            if revert_angle == True:
                if intersect_angle<0:
                    intersect_angle=360+intersect_angle
                else:
                    intersect_angle = -360+intersect_angle
        except:
            pass
        Point_after_rotation = [point_position_after_rotation(after_translate[0], after_translate[1], intersect_angle), target_F2]
        trajectory.append([[(Point_after_rotation[0][0]+target_F1[0])/2, (Point_after_rotation[0][1]+target_F1[1])/2], target_F2])
        trajectory.append([target_F1, target_F2])
    return trajectory

def obtain_feasible_grasp_set(grasp_index_list, current_vertex_position, profile_environment):
    temp_list = grasp_index_list[:]
    for grasp_index in temp_list:
        F1_position = from_index_to_real_position(current_vertex_position, grasp_index[0])
        F2_position = from_index_to_real_position(current_vertex_position, grasp_index[1])
        if Feasible_Grasp(current_vertex_position, F1_position, F2_position, profile_environment)==False:
            grasp_index_list.remove(grasp_index)
            continue      
    return grasp_index_list

def obtain_feasible_grasp_set_of_each_pose(grasp_index_list, object_pose_trajectory, profile_environment, grasp_set_index_memory = dict()):
    feasible_grasp_set_list = []
    for i in range(len(object_pose_trajectory)):
        object_pose_discrete = [[(round(vertex[0]*100)/100), (round(vertex[1]*100)/100)] for vertex in object_pose_trajectory[i]]
        if grasp_set_index_memory.has_key(xtuple(object_pose_discrete)):
            feasible_grasp_set_list.append(grasp_set_index_memory[xtuple(object_pose_discrete)])
        else:
            grasp_index_list_temp = grasp_index_list[:]
            feasible_grasp_set = obtain_feasible_grasp_set(grasp_index_list_temp, object_pose_trajectory[i], profile_environment)
            feasible_grasp_set_list.append(feasible_grasp_set)
            grasp_set_index_memory[xtuple(object_pose_discrete)]=feasible_grasp_set
    return feasible_grasp_set_list, grasp_set_index_memory 

def obtain_suitable_grasp_set_of_each_action(action_sequence, action_parameter_sequence, object_pose_trajectory_plus_ini, feasible_grasp_set_list_plus_ini, ini_vertex_position, ini_CoM_position, profile_environment):
    suitable_grasp_list_sequence = []
    for i in range(len(action_sequence)):
        suitable_grasp_list=[]
        current_feasible_grasp_set = list(set(feasible_grasp_set_list_plus_ini[i]).intersection(set(feasible_grasp_set_list_plus_ini[i+1])))
        for grasp in current_feasible_grasp_set:
            if grasp[0]==0 or grasp[1]==0:
                continue
            F1_position = from_index_to_real_position(object_pose_trajectory_plus_ini[i], grasp[0])
            F2_position = from_index_to_real_position(object_pose_trajectory_plus_ini[i], grasp[1])
            if Suitable_Grasp(object_pose_trajectory_plus_ini[i], F1_position, F2_position, ini_vertex_position, ini_CoM_position, action_sequence[i], action_parameter_sequence[i], profile_environment)==True:
                suitable_grasp_list.append(grasp)
        suitable_grasp_list_sequence.append(suitable_grasp_list)
    return suitable_grasp_list_sequence

def adjust_to_target_grasp(current_vertex_position, current_F1, current_F2, ini_vertex_position, ini_CoM_position, grasp_searching_space, target_grasp_set, target_grasp_position_set, profile_environment, accuracy=0.002):
    if target_grasp_position_set!=None:
        target_grasp_set = []
        for grasp in target_grasp_position_set:
            grasp_index = (from_real_position_to_index(current_vertex_position, grasp[0]), from_real_position_to_index(current_vertex_position, grasp[1]))
            target_grasp_set.append(grasp_index)
    current_CoM = calculate_current_CoM(ini_vertex_position, current_vertex_position, ini_CoM_position)
    current_time0 = time.time()     
    print time.time()-current_time0
    current_F1_index = from_real_position_to_index(current_vertex_position, current_F1)
    current_F2_index = from_real_position_to_index(current_vertex_position, current_F2)
    OPEN_set = {(current_F1_index, current_F2_index)}
    CLOSED = set()
    past_cost=dict.fromkeys(grasp_searching_space, float('inf'))
    past_cost[(current_F1_index, current_F2_index)] = 0
    previous_action = dict()
    parent = dict()
    current_time = time.time()
    while OPEN_set != set():
        if time.time()-current_time>700:
            return 'Infeasible'
        intersection_with_target = list(OPEN_set.intersection(set(target_grasp_set)))
        if intersection_with_target != []:
            final_node=intersection_with_target[0]
            min_past_cost = float('inf')
            for node in intersection_with_target:
                if past_cost[node]<min_past_cost:
                    min_past_cost = past_cost[node]
                    final_node=node
            break
        min_past_cost = past_cost[list(OPEN_set)[0]]
        first_node = list(OPEN_set)[0]
        for node in list(OPEN_set):
            if past_cost[node]<min_past_cost:
                min_past_cost = past_cost[node]
                first_node=node
        print first_node
        time_0 = time.time()
        OPEN_set.remove(first_node)
        if (first_node[0]==0 and first_node[1]!=0) or (first_node[0]!=0 and first_node[1]==0):
            continue
        CLOSED.add(first_node)
        first_node_F1_position = from_index_to_real_position(current_vertex_position, first_node[0])
        first_node_F2_position = from_index_to_real_position(current_vertex_position, first_node[1])
        if first_node in target_grasp_set:
            final_node = first_node
            break
        next_point_list = []
        next_point_action_list = []
        next_point_action_cost_list = []
        if first_node[0] == 0 and first_node[1] == 0:
            for grasp_index in grasp_searching_space:
                if grasp_index[0]>0 and grasp_index[1]>0:
                    grasp_index_F1_position = from_index_to_real_position(current_vertex_position, grasp_index[0])
                    next_point_list.append(grasp_index)
            next_point_action_list = []
            for i in range(len(next_point_list)):
                next_point_action_list.append('MoveGripper')
                next_point_action_cost_list.append(0.01)
        else:
            first_node_F1_position = from_index_to_real_position(current_vertex_position, first_node[0])
            first_node_F2_position = from_index_to_real_position(current_vertex_position, first_node[1]) 

            temp_contact_index, temp_contact_list, temp_contact_normal_list, temp_contact_local_tangent_list = contact_points_and_normal(current_vertex_position, first_node_F1_position, first_node_F2_position, profile_environment)
            Feasible_Action_Set = AdjustGraspAction_feasible_list(temp_contact_index, temp_contact_list, temp_contact_normal_list, temp_contact_local_tangent_list, current_CoM, first_node_F1_position, first_node_F2_position)
            for action in Feasible_Action_Set:
                if action == 'Open':
                    next_point_list.append((0, 0))
                    next_point_action_list.append(action)
                    next_point_action_cost_list.append(0.1)
                else:
                    try:
                        next_finger_position_set = finger_position_after_action(current_vertex_position, first_node_F1_position, first_node_F2_position, action, profile_environment, ini_vertex_position, ini_CoM_position)
                    except:
                        continue
                    if next_finger_position_set==[]:
                        continue
                    for next_finger_couple in next_finger_position_set:
                        [next_F1_position, next_F2_position, action_cost] = next_finger_couple
                        next_F1_index=from_real_position_to_index(current_vertex_position, next_F1_position)     
                        next_F2_index=from_real_position_to_index(current_vertex_position, next_F2_position)
                        if (next_F1_index, next_F2_index) not in grasp_searching_space:
                            continue    
                        if next_F1_index<=0 or next_F2_index<=0:
                            continue        
                        if (next_F1_index != first_node[0] or next_F2_index != first_node[1]):
                            next_point_list.append((next_F1_index, next_F2_index))
                            next_point_action_list.append(action)
                            next_point_action_cost_list.append(action_cost)
        for grasp_number in range(len(next_point_list)):
            if next_point_list[grasp_number] in CLOSED:
                continue
            tentative_past_cost = past_cost[first_node]+next_point_action_cost_list[grasp_number]
            if tentative_past_cost<past_cost[next_point_list[grasp_number]]:
                past_cost[next_point_list[grasp_number]] = tentative_past_cost
                parent[next_point_list[grasp_number]]= first_node
                previous_action[next_point_list[grasp_number]]=next_point_action_list[grasp_number]
                OPEN_set.add(next_point_list[grasp_number])
        print time.time()-time_0
    else:
        return 'Infeasible'

    final_node_temp = final_node
    if target_grasp_position_set != None:
        final_finger_position = target_grasp_position_set[target_grasp_set.index(final_node_temp)]
    previous_action_total = []
    grasp_trajectory_point = []
    while final_node != (current_F1_index, current_F2_index):
        previous_action_total.append(previous_action[final_node])
        grasp_trajectory_point.append(final_node)
        final_node = parent[final_node]
    print time.time()-current_time
    previous_action_total = previous_action_total[::-1]
    grasp_trajectory_point = grasp_trajectory_point[::-1]
    print grasp_trajectory_point
    print previous_action_total
    print current_vertex_position
    finger_position_trajectory = [[current_F1, current_F2]]
    temp_F1_position = current_F1
    temp_F2_position = current_F2
    finish = False
    for i in range(len(previous_action_total)):
        if previous_action_total[i] != 'MoveGripper':
            if i==0:
                finger_position_trajectory += action_trajectory(current_vertex_position, (current_F1_index, current_F2_index), grasp_trajectory_point[i], previous_action_total[i])
            else:
                finger_position_trajectory += action_trajectory(current_vertex_position, grasp_trajectory_point[i-1], grasp_trajectory_point[i], previous_action_total[i])
        else: 
            terminal_F1_position = from_index_to_real_position(current_vertex_position, grasp_trajectory_point[i][0])
            terminal_F2_position = from_index_to_real_position(current_vertex_position, grasp_trajectory_point[i][1])
            if terminal_F1_position != None and terminal_F2_position != None:
                finger_position_trajectory.append([terminal_F1_position, terminal_F2_position])
            elif grasp_trajectory_point[i] == final_node_temp:
                finger_position_trajectory.append(final_finger_position)
    return finger_position_trajectory   

def bottom_layer_grasp_plan(ini_F1, ini_F2, ini_CoM_position, action_sequence, action_parameter_sequence, object_pose_trajectory_plus_ini, feasible_grasp_set_list, suitable_grasp_list_sequence, profile_environment, accuracy=0.002):
    ini_vertex_position = object_pose_trajectory_plus_ini[0]
    total_action_num = len(action_sequence)
    total_obj_pose_num = total_action_num+1
    total_node_set = []
    for i in range(total_obj_pose_num):
        for grasp in feasible_grasp_set_list[i]:
            total_node_set.append((grasp, i))
    target_node_set = []
    for grasp in feasible_grasp_set_list[-1]:
        target_node_set.append((grasp, total_obj_pose_num-1))   
    ini_node = ((from_real_position_to_index(ini_vertex_position, ini_F1), from_real_position_to_index(ini_vertex_position, ini_F2)), 0)
    current_time0 = time.time()     
    OPEN_set = {ini_node}
    CLOSED = set()
    past_cost=dict.fromkeys(total_node_set, float('inf'))
    past_cost[ini_node] = 0
    previous_action = dict()
    parent = dict()
    current_time = time.time()
    while OPEN_set != set():
        if time.time()-current_time>1500:
            return 'Infeasible'
        intersection_with_target = list(OPEN_set.intersection(set(target_node_set)))
        if intersection_with_target != []:
            final_node=intersection_with_target[0]
            min_past_cost = float('inf')
            for node in intersection_with_target:
                if past_cost[node]+(total_obj_pose_num-node[1])<min_past_cost:
                    min_past_cost = past_cost[node]+(total_obj_pose_num-node[1])
                    final_node=node
            break
        min_past_cost = past_cost[list(OPEN_set)[0]]+(total_obj_pose_num-list(OPEN_set)[0][1])
        first_node = list(OPEN_set)[0]
        for node in list(OPEN_set):
            if past_cost[node]+(total_obj_pose_num-node[1])<min_past_cost:
                min_past_cost = past_cost[node]+(total_obj_pose_num-node[1])
                first_node=node
        print first_node
        time_0 = time.time()
        OPEN_set.remove(first_node)
        if (first_node[0][0]==0 and first_node[0][1]!=0) or (first_node[0][0]!=0 and first_node[0][1]==0):
            continue
        CLOSED.add(first_node)
        first_node_F1_position = from_index_to_real_position(object_pose_trajectory_plus_ini[first_node[1]], first_node[0][0])
        first_node_F2_position = from_index_to_real_position(object_pose_trajectory_plus_ini[first_node[1]], first_node[0][1])
        if first_node in target_node_set:
            final_node = first_node
            break
        next_node_list = []
        next_node_action_list = []
        next_node_action_cost_list = []
        if first_node[0][0] == 0 and first_node[0][1] == 0:
            for grasp in feasible_grasp_set_list[first_node[1]]:
                if grasp[0]>0 and grasp[1]>0:
                    next_node_list.append((grasp, first_node[1]))
            next_node_action_list = []
            for i in range(len(next_node_list)):
                next_node_action_list.append('MoveGripper')
                next_node_action_cost_list.append(0.2)
        else:
            current_vertex_position = object_pose_trajectory_plus_ini[first_node[1]]
            first_node_F1_position = from_index_to_real_position(current_vertex_position, first_node[0][0])
            first_node_F2_position = from_index_to_real_position(current_vertex_position, first_node[0][1]) 
            temp_contact_index, temp_contact_list, temp_contact_normal_list, temp_contact_local_tangent_list = contact_points_and_normal(current_vertex_position, first_node_F1_position, first_node_F2_position, profile_environment)
            current_CoM = calculate_current_CoM(ini_vertex_position, current_vertex_position, ini_CoM_position)
            Feasible_Action_Set = AdjustGraspAction_feasible_list(temp_contact_index, temp_contact_list, temp_contact_normal_list, temp_contact_local_tangent_list, current_CoM, first_node_F1_position, first_node_F2_position)
            for action in Feasible_Action_Set:
                if action == 'Open':
                    next_node_list.append(((0, 0), first_node[1]))
                    next_node_action_list.append(action)
                    next_node_action_cost_list.append(0.1)
                else:
                    try:
                        next_finger_position_set = finger_position_after_action(current_vertex_position, first_node_F1_position, first_node_F2_position, action, profile_environment, ini_vertex_position, ini_CoM_position)
                    except:
                        continue
                    if next_finger_position_set==[]:
                        continue
                    for next_finger_couple in next_finger_position_set:
                        [next_F1_position, next_F2_position, action_cost] = next_finger_couple
                        next_F1_index=from_real_position_to_index(current_vertex_position, next_F1_position)     
                        next_F2_index=from_real_position_to_index(current_vertex_position, next_F2_position)
                        if (next_F1_index, next_F2_index) not in feasible_grasp_set_list[first_node[1]]:
                            continue    
                        if next_F1_index<=0 or next_F2_index<=0:
                            continue        
                        if (next_F1_index != first_node[0][0] or next_F2_index != first_node[0][1]):
                            next_node_list.append(((next_F1_index, next_F2_index), first_node[1]))
                            next_node_action_list.append(action)
                            next_node_action_cost_list.append(action_cost)
            if first_node[1] != total_obj_pose_num-1 and first_node[0] in suitable_grasp_list_sequence[first_node[1]]:
                next_node_list.append((first_node[0], first_node[1]+1))
                next_node_action_list.append(action_sequence[first_node[1]])
                next_node_action_cost_list.append(0.01)
                
        for node_number in range(len(next_node_list)):
            if next_node_list[node_number] in CLOSED:
                continue
            tentative_past_cost = past_cost[first_node]+next_node_action_cost_list[node_number]
            if tentative_past_cost<past_cost[next_node_list[node_number]]:
                past_cost[next_node_list[node_number]] = tentative_past_cost
                parent[next_node_list[node_number]]= first_node
                previous_action[next_node_list[node_number]]=next_node_action_list[node_number]
                OPEN_set.add(next_node_list[node_number])
        print time.time()-time_0
    else:
        return 'Infeasible'

    final_node_temp = final_node
    previous_action_total = []
    grasp_trajectory_node_sequence = []
    while final_node != ini_node:
        previous_action_total.append(previous_action[final_node])
        grasp_trajectory_node_sequence.append(final_node)
        final_node = parent[final_node]
    grasp_trajectory_node_sequence.append(ini_node)
    print time.time()-current_time
    previous_action_total = previous_action_total[::-1]
    grasp_trajectory_node_sequence = grasp_trajectory_node_sequence[::-1]
    print grasp_trajectory_node_sequence
    print previous_action_total
    finger_position_trajectory = [[ini_F1, ini_F2]]
    object_pose_trajectory = [object_pose_trajectory_plus_ini[0]]
    for node_index in range(len(grasp_trajectory_node_sequence)):
        segment_ini_F1_position = from_index_to_real_position(object_pose_trajectory_plus_ini[grasp_trajectory_node_sequence[node_index][1]], grasp_trajectory_node_sequence[node_index][0][0])
        segment_ini_F2_position = from_index_to_real_position(object_pose_trajectory_plus_ini[grasp_trajectory_node_sequence[node_index][1]], grasp_trajectory_node_sequence[node_index][0][1])
        if node_index != len(grasp_trajectory_node_sequence)-1 and grasp_trajectory_node_sequence[node_index][1]==grasp_trajectory_node_sequence[node_index+1][1]:
            if previous_action_total[node_index] != 'MoveGripper':
                finger_position_trajectory_segment = [[segment_ini_F1_position, segment_ini_F2_position]]
                segment_tar_F1_position = from_index_to_real_position(object_pose_trajectory_plus_ini[grasp_trajectory_node_sequence[node_index][1]], grasp_trajectory_node_sequence[node_index+1][0][0])
                segment_tar_F2_position = from_index_to_real_position(object_pose_trajectory_plus_ini[grasp_trajectory_node_sequence[node_index][1]], grasp_trajectory_node_sequence[node_index+1][0][1])
                finger_position_trajectory_segment += action_trajectory(object_pose_trajectory_plus_ini[grasp_trajectory_node_sequence[node_index][1]], grasp_trajectory_node_sequence[node_index][0], grasp_trajectory_node_sequence[node_index+1][0], previous_action_total[node_index])
                finger_position_trajectory += finger_position_trajectory_segment
                for i in range(len(finger_position_trajectory_segment)):
                    object_pose_trajectory.append(object_pose_trajectory_plus_ini[grasp_trajectory_node_sequence[node_index][1]])
            else:
                segment_tar_F1_position = from_index_to_real_position(object_pose_trajectory_plus_ini[grasp_trajectory_node_sequence[node_index][1]], grasp_trajectory_node_sequence[node_index+1][0][0])
                segment_tar_F2_position = from_index_to_real_position(object_pose_trajectory_plus_ini[grasp_trajectory_node_sequence[node_index][1]], grasp_trajectory_node_sequence[node_index+1][0][1])      
                finger_position_trajectory +=  [[segment_tar_F1_position, segment_tar_F2_position]]
                object_pose_trajectory.append(object_pose_trajectory_plus_ini[grasp_trajectory_node_sequence[node_index][1]])
        elif node_index != len(grasp_trajectory_node_sequence)-1 and grasp_trajectory_node_sequence[node_index][1]!=grasp_trajectory_node_sequence[node_index+1][1]:
            temp_object_pose_list, temp_F1_list, temp_F2_list = system_config_list(object_pose_trajectory_plus_ini[grasp_trajectory_node_sequence[node_index][1]], segment_ini_F1_position, segment_ini_F2_position, action_sequence[grasp_trajectory_node_sequence[node_index][1]], action_parameter_sequence[grasp_trajectory_node_sequence[node_index][1]], profile_environment)
            object_pose_trajectory += temp_object_pose_list
            finger_position_trajectory += [[temp_F1_list[i], temp_F2_list[i]] for i in range(len(temp_object_pose_list))]
    return finger_position_trajectory, object_pose_trajectory 
