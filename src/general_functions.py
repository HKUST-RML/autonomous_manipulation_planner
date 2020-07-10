#!/usr/bin/env python
import numpy as np
import re
import gol
from math import *
from scipy.optimize import linprog
from jerk import *
from shapely.geometry import LineString
from shapely.geometry import Polygon
from shapely.geometry import Point
from shapely.geometry import LinearRing

def CtoAB(A,B,C):
    CA=list(map(lambda x: x[0]-x[1], zip(A, C)))
    CB=list(map(lambda x: x[0]-x[1], zip(B, C)))
    ABdistance = np.sqrt((A[0]-B[0])**2+(A[1]-B[1])**2)
    return abs(np.cross(CA,CB))/ABdistance

def calculate_intersect_angle_AB_CD(A, B, C, D):
    AB=list(map(lambda x: x[0]-x[1], zip(A, B)))
    CD=list(map(lambda x: x[0]-x[1], zip(C, D)))
    ABdistance = np.sqrt((A[0]-B[0])**2+(A[1]-B[1])**2)
    CDdistance = np.sqrt((C[0]-D[0])**2+(C[1]-D[1])**2)
    cos_angle = np.dot(AB,CD)/ABdistance/CDdistance
    sin_angle = np.cross(AB,CD)/ABdistance/CDdistance
    if abs(np.dot(AB,CD))<(ABdistance*CDdistance):
        if cos_angle>=0 and sin_angle>=0:
            tar_angle = acos(cos_angle)
        elif cos_angle>=0 and sin_angle<0:
            tar_angle = -acos(cos_angle)
        elif cos_angle<0 and sin_angle>=0:
            tar_angle = acos(cos_angle)
        else:
            tar_angle = -acos(cos_angle)
        return tar_angle*180/pi
    else:
        return 0

def calculate_intersect_point_AB_CD(A,B,C,D):
    T1 = [[B[1]-A[1], A[0]-B[0]], [D[1] - C[1], C[0] - D[0]]]
    T2 = [[-(B[0]-A[0])*A[1]+(B[1]-A[1])*A[0]], [-(D[0]-C[0])*C[1]+(D[1]-C[1])*C[0]]]
    try:
        return np.squeeze(np.linalg.solve(T1,T2)).tolist()
    except BaseException:
        return False

def calculate_current_CoM(ini_vertex_position, current_vertex_position, ini_CoM_position):
    current_CoM_position_matrix = np.dot(np.dot(np.array([[current_vertex_position[0][0], current_vertex_position[0][1], 1], [current_vertex_position[1][0], current_vertex_position[1][1], 1], [current_vertex_position[2][0], current_vertex_position[2][1], 1]]).T, np.linalg.inv(np.array([[ini_vertex_position[0][0], ini_vertex_position[0][1], 1], [ini_vertex_position[1][0], ini_vertex_position[1][1], 1], [ini_vertex_position[2][0], ini_vertex_position[2][1], 1]]).T)), np.array([[ini_CoM_position[0]], [ini_CoM_position[1]], [1]])).tolist()
    current_CoM_position = [current_CoM_position_matrix[0][0], current_CoM_position_matrix[1][0]]
    return current_CoM_position

def footCtoAB(A,B,C):
    k=float(((C[0]-A[0])*(B[0]-A[0])+(C[1]-A[1])*(B[1]-A[1])))/float(((B[0]-A[0])*(B[0]-A[0])+(B[1]-A[1])*(B[1]-A[1])))
    return [A[0]+k*(B[0]-A[0]), A[1]+k*(B[1]-A[1])]

def calculate_parameter_translate_and_rotate(ini_A, ini_B, tar_A, tar_B):  
    delta_translate = [tar_A[0]-ini_A[0], tar_A[1]-ini_A[1]]
    tar_shift_A = [tar_A[0]-delta_translate[0], tar_A[1]-delta_translate[1]]
    tar_shift_B = [tar_B[0]-delta_translate[0], tar_B[1]-delta_translate[1]]
    P_r = ini_A
    A_radius = (np.sqrt((ini_B[0]-P_r[0])**2+(ini_B[1]-P_r[1])**2)+np.sqrt((tar_shift_B[0]-P_r[0])**2+(tar_shift_B[1]-P_r[1])**2))/2
    cos_ini_angle0 = max(min((ini_B[0] - P_r[0])/A_radius,1),-1)  
    sin_ini_angle0 = max(min((ini_B[1] - P_r[1])/A_radius,1),-1) 
    if cos_ini_angle0>=0 and sin_ini_angle0>=0:
        ini_angle0 = acos(cos_ini_angle0)
    elif cos_ini_angle0>=0 and sin_ini_angle0<0:
        ini_angle0 = -acos(cos_ini_angle0)
    elif cos_ini_angle0<0 and sin_ini_angle0>=0:
        ini_angle0 = acos(cos_ini_angle0)
    else:
        ini_angle0 = -acos(cos_ini_angle0)
    cos_tar_angle = max(min((tar_shift_B[0] - P_r[0])/A_radius,1),-1)   
    sin_tar_angle = max(min((tar_shift_B[1] - P_r[1])/A_radius,1),-1)   
    if cos_tar_angle>=0 and sin_tar_angle>=0:
        tar_angle = acos(cos_tar_angle)
    elif cos_tar_angle>=0 and sin_tar_angle<0:
        tar_angle = -acos(cos_tar_angle)
    elif cos_tar_angle<0 and sin_tar_angle>=0:
        tar_angle = acos(cos_tar_angle)
    else:
        tar_angle = -acos(cos_tar_angle)
    A_r = tar_angle-ini_angle0
    if A_r > 0:
        D_r = 'counterclockwise'
    else:
        D_r = 'clockwise'
    return delta_translate, D_r, A_r

def calculate_parameter_translate_and_rotate_moveB(ini_A, ini_B, tar_A, tar_B):  
    delta_translate = [tar_B[0]-ini_B[0], tar_B[1]-ini_B[1]]
    tar_shift_A = [tar_A[0]-delta_translate[0], tar_A[1]-delta_translate[1]]
    tar_shift_B = [tar_B[0]-delta_translate[0], tar_B[1]-delta_translate[1]]
    P_r = ini_B
    B_radius = (np.sqrt((ini_A[0]-P_r[0])**2+(ini_A[1]-P_r[1])**2)+np.sqrt((tar_shift_A[0]-P_r[0])**2+(tar_shift_A[1]-P_r[1])**2))/2
    cos_ini_angle0 = max(min((ini_A[0] - P_r[0])/B_radius,1),-1)  
    sin_ini_angle0 = max(min((ini_A[1] - P_r[1])/B_radius,1),-1) 
    if cos_ini_angle0>=0 and sin_ini_angle0>=0:
        ini_angle0 = acos(cos_ini_angle0)
    elif cos_ini_angle0>=0 and sin_ini_angle0<0:
        ini_angle0 = -acos(cos_ini_angle0)
    elif cos_ini_angle0<0 and sin_ini_angle0>=0:
        ini_angle0 = acos(cos_ini_angle0)
    else:
        ini_angle0 = -acos(cos_ini_angle0)
    cos_tar_angle = max(min((tar_shift_A[0] - P_r[0])/B_radius,1),-1)   
    sin_tar_angle = max(min((tar_shift_A[1] - P_r[1])/B_radius,1),-1)   
    if cos_tar_angle>=0 and sin_tar_angle>=0:
        tar_angle = acos(cos_tar_angle)
    elif cos_tar_angle>=0 and sin_tar_angle<0:
        tar_angle = -acos(cos_tar_angle)
    elif cos_tar_angle<0 and sin_tar_angle>=0:
        tar_angle = acos(cos_tar_angle)
    else:
        tar_angle = -acos(cos_tar_angle)
    A_r = tar_angle-ini_angle0
    if A_r > 0:
        D_r = 'counterclockwise'
    else:
        D_r = 'clockwise'
    return delta_translate, D_r, A_r

def generate_vector(A, B):
    return [B[0]-A[0], B[1]-A[1]]

def point_position_after_rotation(current_xy, rotation_pole, desired_angle):
    desired_angle_rad = desired_angle*pi/180
    current_displacement = list(np.array(current_xy)-np.array(rotation_pole))
    rotation_matrix = [[cos(desired_angle_rad), -sin(desired_angle_rad)],[sin(desired_angle_rad), cos(desired_angle_rad)]]
    current_displacement = np.expand_dims(current_displacement, axis=1)
    temp = np.dot(rotation_matrix, current_displacement)
    xy_after_rotate = [list(rotation_pole[0]+temp[0])[0], list(rotation_pole[1]+temp[1])[0]]
    return xy_after_rotate 

def calculate_CoM(current_vertex_point):
    area = 0.0
    x,y = 0.0,0.0
    a = len(current_vertex_point)
    for i in range(a):
        lat = current_vertex_point[i][0]
        lng = current_vertex_point[i][1]
        if i == 0:
            lat1 = current_vertex_point[-1][0]
            lng1 = current_vertex_point[-1][1]
        else:
            lat1 = current_vertex_point[i-1][0]
            lng1 = current_vertex_point[i-1][1]
        fg = (lat*lng1 - lng*lat1)/2.0
        area += fg
        x += fg*(lat+lat1)/3.0
        y += fg*(lng+lng1)/3.0
    x = x/area
    y = y/area
    return x,y

def contact_points_and_normal(current_vertex_position, F1, F2, profile_environment, accuracy = 0.01):
    geometric_center = (sum(np.array(current_vertex_position).astype(float))/len(current_vertex_position)).tolist()
    contact_index = []
    contact_list = []
    contact_normal_list = []
    contact_local_tangent_list = []
    index = 1
    for i in range(len(current_vertex_position)):
        for j in range(len(profile_environment)):
            if j!=0 and j!=len(profile_environment)-1 and Point(current_vertex_position[i]).distance(Point(profile_environment[j])) <accuracy:
                contact_point = current_vertex_position[i]
                contact_center = list(map(lambda x: x[0]-x[1], zip(geometric_center, contact_point)))
                contact_edge_length0 = LineString([profile_environment[j-1], profile_environment[j]]).length
                contact_edge_length1 = LineString([profile_environment[j], profile_environment[j+1]]).length
                contact_normal0 = [(profile_environment[j][1]-profile_environment[j-1][1])/contact_edge_length0, -(profile_environment[j][0]-profile_environment[j-1][0])/contact_edge_length0]
                contact_normal1 = [(profile_environment[j+1][1]-profile_environment[j][1])/contact_edge_length1, -(profile_environment[j+1][0]-profile_environment[j][0])/contact_edge_length1]
                if np.dot(contact_center,contact_normal0)<0:
                    contact_normal0 = [-contact_normal0[0], -contact_normal0[1]]
                if np.dot(contact_center,contact_normal1)<0:
                    contact_normal1 = [-contact_normal1[0], -contact_normal1[1]]
                for point in contact_list:
                    if Point(contact_point).distance(Point(point))<accuracy:
                        contact_normal_list[contact_list.index(point)] = [contact_normal0,contact_normal1]
                        contact_local_tangent_list[contact_list.index(point)] = [profile_environment[j-1], profile_environment[j], profile_environment[j+1]]
                        break
                else:
                    contact_list.append(contact_point)
                    contact_index.append('E'+str(index))
                    index = index+1
                    contact_normal_list.append([contact_normal0,contact_normal1])
                    contact_local_tangent_list.append([profile_environment[j-1], profile_environment[j], profile_environment[j+1]])
            if j!=len(profile_environment)-1 and Point(current_vertex_position[i]).distance(LineString([profile_environment[j], profile_environment[j+1]]))<accuracy:
                contact_point = current_vertex_position[i]
                for point in contact_list:
                    if Point(contact_point).distance(Point(point))<accuracy:
                        break
                else:
                    contact_list.append(contact_point)
                    contact_index.append('E'+str(index))
                    index = index+1
                    contact_center = list(map(lambda x: x[0]-x[1], zip(geometric_center, contact_point)))
                    contact_edge_length = LineString([profile_environment[j], profile_environment[j+1]]).length
                    contact_normal = [(profile_environment[j+1][1]-profile_environment[j][1])/contact_edge_length, -(profile_environment[j+1][0]-profile_environment[j][0])/contact_edge_length]
                    if np.dot(contact_center,contact_normal)<0:
                        contact_normal = [-contact_normal[0], -contact_normal[1]]
                    contact_normal_list.append(contact_normal)
                    contact_local_tangent_list.append([profile_environment[j], profile_environment[j+1]])
            if Point(profile_environment[j]).distance(LineString([current_vertex_position[i], current_vertex_position[(i+1)%(len(current_vertex_position))]]))<accuracy:
                contact_point = footCtoAB(current_vertex_position[i], current_vertex_position[(i+1)%(len(current_vertex_position))], profile_environment[j])
                for point in contact_list:
                    if Point(contact_point).distance(Point(point))<accuracy:
                        break
                else:
                    contact_list.append(contact_point)
                    contact_index.append('E'+str(index))
                    index = index+1
                    contact_center = list(map(lambda x: x[0]-x[1], zip(geometric_center, contact_point)))
                    contact_edge_length = LineString([current_vertex_position[i], current_vertex_position[(i+1)%(len(current_vertex_position))]]).length
                    contact_normal = [(current_vertex_position[(i+1)%(len(current_vertex_position))][1]-current_vertex_position[i][1])/contact_edge_length, -(current_vertex_position[(i+1)%(len(current_vertex_position))][0]-current_vertex_position[i][0])/contact_edge_length]
                    if np.dot(contact_center,contact_normal)<0:
                        contact_normal = [-contact_normal[0], -contact_normal[1]]
                    contact_normal_list.append(contact_normal)
                    contact_local_tangent_list.append([current_vertex_position[i], current_vertex_position[(i+1)%(len(current_vertex_position))]])
        if F1 != None and Point(F1).distance(LineString([current_vertex_position[i],current_vertex_position[(i+1)%(len(current_vertex_position))]]))<accuracy and 'F1' not in contact_index:
            contact_list.append(footCtoAB(current_vertex_position[i],current_vertex_position[(i+1)%(len(current_vertex_position))],F1))
            contact_index.append('F1')
            contact_edge_length = LineString([current_vertex_position[i], current_vertex_position[(i+1)%(len(current_vertex_position))]]).length
            F1_normal = [(current_vertex_position[(i+1)%(len(current_vertex_position))][1]-current_vertex_position[i][1])/contact_edge_length, -(current_vertex_position[(i+1)%(len(current_vertex_position))][0]-current_vertex_position[i][0])/contact_edge_length]
            F1_center = list(map(lambda x: x[0]-x[1], zip(geometric_center, F1)))
            if np.dot(F1_center,F1_normal)<0:
                F1_normal = [-F1_normal[0], -F1_normal[1]]
            contact_normal_list.append(F1_normal)
            contact_local_tangent_list.append([current_vertex_position[i], current_vertex_position[(i+1)%(len(current_vertex_position))]])
        if F2 != None and Point(F2).distance(LineString([current_vertex_position[i],current_vertex_position[(i+1)%(len(current_vertex_position))]]))<accuracy and 'F2' not in contact_index:
            contact_list.append(footCtoAB(current_vertex_position[i],current_vertex_position[(i+1)%(len(current_vertex_position))],F2))
            contact_index.append('F2')
            contact_edge_length = LineString([current_vertex_position[i], current_vertex_position[(i+1)%(len(current_vertex_position))]]).length
            F2_normal = [(current_vertex_position[(i+1)%(len(current_vertex_position))][1]-current_vertex_position[i][1])/contact_edge_length, -(current_vertex_position[(i+1)%(len(current_vertex_position))][0]-current_vertex_position[i][0])/contact_edge_length]
            F2_center = list(map(lambda x: x[0]-x[1], zip(geometric_center, F2)))
            if np.dot(F2_center,F2_normal)<0:
                F2_normal = [-F2_normal[0], -F2_normal[1]]
            contact_normal_list.append(F2_normal)
            contact_local_tangent_list.append([current_vertex_position[i], current_vertex_position[(i+1)%(len(current_vertex_position))]])
    return contact_index, contact_list, contact_normal_list, contact_local_tangent_list

def contact_points_and_normal_finger(current_vertex_position, F1, F2, profile_environment, accuracy = 0.01):
    geometric_center = (sum(np.array(current_vertex_position).astype(float))/len(current_vertex_position)).tolist()
    contact_index = []
    contact_list = []
    contact_normal_list = []
    contact_local_tangent_list = []
    index = 1
    for i in range(len(current_vertex_position)):
        if F1 != None and Point(F1).distance(LineString([current_vertex_position[i],current_vertex_position[(i+1)%(len(current_vertex_position))]]))<accuracy and 'F1' not in contact_index:
            contact_list.append(footCtoAB(current_vertex_position[i],current_vertex_position[(i+1)%(len(current_vertex_position))],F1))
            contact_index.append('F1')
            contact_edge_length = LineString([current_vertex_position[i], current_vertex_position[(i+1)%(len(current_vertex_position))]]).length
            F1_normal = [(current_vertex_position[(i+1)%(len(current_vertex_position))][1]-current_vertex_position[i][1])/contact_edge_length, -(current_vertex_position[(i+1)%(len(current_vertex_position))][0]-current_vertex_position[i][0])/contact_edge_length]
            F1_center = list(map(lambda x: x[0]-x[1], zip(geometric_center, F1)))
            if np.dot(F1_center,F1_normal)<0:
                F1_normal = [-F1_normal[0], -F1_normal[1]]
            contact_normal_list.append(F1_normal)
            contact_local_tangent_list.append([current_vertex_position[i], current_vertex_position[(i+1)%(len(current_vertex_position))]])
        if F2 != None and Point(F2).distance(LineString([current_vertex_position[i],current_vertex_position[(i+1)%(len(current_vertex_position))]]))<accuracy and 'F2' not in contact_index:
            contact_list.append(footCtoAB(current_vertex_position[i],current_vertex_position[(i+1)%(len(current_vertex_position))],F2))
            contact_index.append('F2')
            contact_edge_length = LineString([current_vertex_position[i], current_vertex_position[(i+1)%(len(current_vertex_position))]]).length
            F2_normal = [(current_vertex_position[(i+1)%(len(current_vertex_position))][1]-current_vertex_position[i][1])/contact_edge_length, -(current_vertex_position[(i+1)%(len(current_vertex_position))][0]-current_vertex_position[i][0])/contact_edge_length]
            F2_center = list(map(lambda x: x[0]-x[1], zip(geometric_center, F2)))
            if np.dot(F2_center,F2_normal)<0:
                F2_normal = [-F2_normal[0], -F2_normal[1]]
            contact_normal_list.append(F2_normal)
            contact_local_tangent_list.append([current_vertex_position[i], current_vertex_position[(i+1)%(len(current_vertex_position))]])
    return contact_index, contact_list, contact_normal_list, contact_local_tangent_list

def circumference(current_vertex_position):
    accumulate_length = LinearRing(current_vertex_position).length
    return accumulate_length

def finger_position2index(current_vertex_position, finger_position, accuracy=0.01):
    finger_position_index = LinearRing(current_vertex_position).project(Point(finger_position))
    return finger_position_index

def finger_index2position(current_vertex_position, current_position_index):
    accumulate_length = 0
    finger_position=list(list(LinearRing(current_vertex_position).interpolate(current_position_index).coords)[0])
    return finger_position

def friction_tangent_direction(contact_normal, contact_mode, contact_local_tangent):
    if len(contact_local_tangent) == 2:
        if contact_mode == 'S+':
            if np.cross([-contact_normal[0], -contact_normal[1]], generate_vector(contact_local_tangent[0], contact_local_tangent[1]))>0:
                friction_tangent_direction = generate_vector(contact_local_tangent[0], contact_local_tangent[1])
            else:
                friction_tangent_direction = generate_vector(contact_local_tangent[1], contact_local_tangent[0])
        else:
            if np.cross([-contact_normal[0], -contact_normal[1]], generate_vector(contact_local_tangent[0], contact_local_tangent[1]))<0:
                friction_tangent_direction = generate_vector(contact_local_tangent[0], contact_local_tangent[1])
            else:
                friction_tangent_direction = generate_vector(contact_local_tangent[1], contact_local_tangent[0])
    else:
        if contact_mode == 'S+':
            if np.cross([-contact_normal[0][0], -contact_normal[0][1]], generate_vector(contact_local_tangent[0], contact_local_tangent[1]))>0:
                friction_tangent_direction = generate_vector(contact_local_tangent[0], contact_local_tangent[1])
            else:
                friction_tangent_direction = generate_vector(contact_local_tangent[2], contact_local_tangent[1])
        else:
            if np.cross([-contact_normal[0][0], -contact_normal[0][1]], generate_vector(contact_local_tangent[0], contact_local_tangent[1]))<0:
                friction_tangent_direction = generate_vector(contact_local_tangent[0], contact_local_tangent[1])
            else:
                friction_tangent_direction = generate_vector(contact_local_tangent[2], contact_local_tangent[1])
    friction_tangent_direction_length = sqrt(friction_tangent_direction[0]**2+friction_tangent_direction[1]**2)
    friction_tangent_direction = [friction_tangent_direction[0]/friction_tangent_direction_length, friction_tangent_direction[1]/friction_tangent_direction_length]
    return friction_tangent_direction

def whether_stable(contact_index, contact_list, contact_normal_list, contact_local_tangent_list, current_CoM, contact_mode, mu_gripper=0.6, mu_environment=0.03):
    mu_gripper=gol.get_value('mu_gripper', 0.6)
    mu_environment=gol.get_value('mu_environment', 0.03)
    IsStable = False
    contact_normal_and_gravity=np.expand_dims([np.cross(current_CoM, [0,-1]), 0, -1], axis=1)
    for i in range(len(contact_index)):
        if re.match('E', contact_index[i])!=None:
            mu=mu_environment
        else:
            mu=mu_gripper
        if contact_mode.has_key(contact_index[i]) and (contact_mode[contact_index[i]] == 'R'):
            if len(np.array(contact_normal_list[i]).shape) == 1:
                fri_cone1 = np.dot([[cos(atan(-mu)), -sin(atan(-mu))],[sin(atan(-mu)), cos(atan(-mu))]], np.expand_dims(contact_normal_list[i], axis=1)).transpose()[0]
                fri_cone2 = np.dot([[cos(atan(mu)), -sin(atan(mu))],[sin(atan(mu)), cos(atan(mu))]], np.expand_dims(contact_normal_list[i], axis=1)).transpose()[0]
                fri_wrench1 = np.expand_dims([np.cross(contact_list[i], fri_cone1), fri_cone1[0], fri_cone1[1]], axis=1)
                fri_wrench2 = np.expand_dims([np.cross(contact_list[i], fri_cone2), fri_cone2[0], fri_cone2[1]], axis=1)
                contact_normal_and_gravity = np.hstack((contact_normal_and_gravity, fri_wrench1, fri_wrench2))
            else:
                for contact_normal in contact_normal_list[i]:
                    fri_cone1 = np.dot([[cos(atan(-mu)), -sin(atan(-mu))],[sin(atan(-mu)), cos(atan(-mu))]], np.expand_dims(contact_normal, axis=1)).transpose()[0]
                    fri_cone2 = np.dot([[cos(atan(mu)), -sin(atan(mu))],[sin(atan(mu)), cos(atan(mu))]], np.expand_dims(contact_normal, axis=1)).transpose()[0]
                    fri_wrench1 = np.expand_dims([np.cross(contact_list[i], fri_cone1), fri_cone1[0], fri_cone1[1]], axis=1)
                    fri_wrench2 = np.expand_dims([np.cross(contact_list[i], fri_cone2), fri_cone2[0], fri_cone2[1]], axis=1)
                    contact_normal_and_gravity = np.hstack((contact_normal_and_gravity, fri_wrench1, fri_wrench2))
        elif (contact_mode.has_key(contact_index[i]) and contact_mode[contact_index[i]] == 'S+') or (contact_mode.has_key(contact_index[i]) and contact_mode[contact_index[i]] == 'S-'):
            fri_cone_list = []
            friction_tangent = friction_tangent_direction(contact_normal_list[i], contact_mode[contact_index[i]], contact_local_tangent_list[i])
            if len(np.array(contact_normal_list[i]).shape) == 1:
                fri_cone_list.append(np.dot([[cos(atan(-mu)), -sin(atan(-mu))],[sin(atan(-mu)), cos(atan(-mu))]], np.expand_dims(contact_normal_list[i], axis=1)).transpose()[0])
                fri_cone_list.append(np.dot([[cos(atan(mu)), -sin(atan(mu))],[sin(atan(mu)), cos(atan(mu))]], np.expand_dims(contact_normal_list[i], axis=1)).transpose()[0])
            else:
                for contact_normal in contact_normal_list[i]:
                    fri_cone_list.append(np.dot([[cos(atan(-mu)), -sin(atan(-mu))],[sin(atan(-mu)), cos(atan(-mu))]], np.expand_dims(contact_normal, axis=1)).transpose()[0])
                    fri_cone_list.append(np.dot([[cos(atan(mu)), -sin(atan(mu))],[sin(atan(mu)), cos(atan(mu))]], np.expand_dims(contact_normal, axis=1)).transpose()[0])
            for fri_cone in fri_cone_list:
                if np.dot(fri_cone, friction_tangent)>0 and abs(np.dot(fri_cone, friction_tangent)/sqrt(1-(np.dot(fri_cone, friction_tangent))**2)-mu)<0.001:
                    fri_wrench = np.expand_dims([np.cross(contact_list[i], fri_cone), fri_cone[0], fri_cone[1]], axis=1)
                    contact_normal_and_gravity = np.hstack((contact_normal_and_gravity, fri_wrench)) 
                    break           
    if np.linalg.matrix_rank(contact_normal_and_gravity) == 3:          
        f = list(np.ones(contact_normal_and_gravity.shape[1]))
        res = linprog(f, A_eq=contact_normal_and_gravity, b_eq=np.zeros(3), bounds=(1, None))
        if res.status == 0:
            IsStable = True
    return IsStable 
