# Copyright 2020 Technical University of Munich, Professorship of Cyber-Physical Systems, Matthew O'Kelly, Aman Sinha, Hongrui Zheng

# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.



"""
Prototype of vehicle dynamics functions and classes for simulating 2D Single 
Track dynamic model
Following the implementation of commanroad's Single Track Dynamics model
Original implementation: https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/
Author: Hongrui Zheng
"""

from os import X_OK
import numpy as np
from numba import njit

import unittest
import time
import math

oldU = np.zeros((1,2),dtype=float)
prev_ey = 0
sum_ey = 0

waypoints = np.loadtxt("IMS_map_waypoints.csv", delimiter=",")

# @njit(cache=True)
def getCurvature(X,Y):
    global waypoints
    position = np.array([X,Y])
    # waypoints = np.loadtxt("IMS_map_waypoints.csv", delimiter=",")
    # waypoints = np.loadtxt("IMS_raceline.csv", delimiter=';', skiprows=3)

    wpts = np.vstack((waypoints[:, 0], waypoints[:, 1])).T
    _,_,_,nearest_point_id,_ = nearest_point_on_trajectory(position, wpts)
    cur = waypoints[nearest_point_id,4]
    # print("{:.3f}".format(((X-waypoints[nearest_point_id,0])**2 + (Y-waypoints[nearest_point_id,1])**2)**0.5))
    return cur,nearest_point_id,wpts

def getCurvature_S(s):
    global waypoints
    diff_args = np.argsort(np.abs(waypoints[:,5]-s))
    curv = ((s-waypoints[diff_args[0],5])/(waypoints[diff_args[1],5] - waypoints[diff_args[0],5]))\
            *(waypoints[diff_args[1],4] - waypoints[diff_args[0],4]) + waypoints[diff_args[0],4]
    return curv
    
@njit(fastmath=False, cache=True)
def nearest_point_on_trajectory(point, trajectory):
    '''
    Return the nearest point along the given piecewise linear trajectory.

    Same as nearest_point_on_line_segment, but vectorized. This method is quite fast, time constraints should
    not be an issue so long as trajectories are not insanely long.

        Order of magnitude: trajectory length: 1000 --> 0.0002 second computation (5000fps)

    point: size 2 numpy array
    trajectory: Nx2 matrix of (x,y) trajectory waypoints
        - these must be unique. If they are not unique, a divide by 0 error will destroy the world
    '''
    diffs = trajectory[1:,:] - trajectory[:-1,:]
    l2s   = diffs[:,0]**2 + diffs[:,1]**2
    # this is equivalent to the elementwise dot product
    dots = np.empty((trajectory.shape[0]-1, ))
    for i in range(dots.shape[0]):
        dots[i] = np.dot((point - trajectory[i, :]), diffs[i, :])
    t = dots / l2s
    t[t<0.0] = 0.0
    t[t>1.0] = 1.0

    projections = trajectory[:-1,:] + (t*diffs.T).T

    dists = np.empty((projections.shape[0],))
    for i in range(dists.shape[0]):
        temp = point - projections[i]
        dists[i] = np.sqrt(np.sum(temp*temp))
    min_dist_segment = np.argmin(dists)
    
    dists_args_sorted = np.argsort(dists)
    slope = (trajectory[dists_args_sorted[0],1]-trajectory[dists_args_sorted[1],1])/(trajectory[dists_args_sorted[0],0]-trajectory[dists_args_sorted[1],0])
    perpendicular_slope = -1/slope
    
    intersected_point = np.zeros(2)
    intersected_point[0] = (point[1] - trajectory[min_dist_segment,1] + slope * trajectory[min_dist_segment,0] -\
                                perpendicular_slope*point[0])/(slope-perpendicular_slope)
    intersected_point[1] = point[1] + perpendicular_slope*(intersected_point[0]-point[0])
    
    return projections[min_dist_segment], dists[min_dist_segment], t[min_dist_segment], min_dist_segment,intersected_point


def vehicle_dynamics_st(x, x_glob, u):

    m  = 1.98
    lf = 0.125
    lr = 0.125
    Iz = 0.024
    Df = 0.8 * m * 9.81 / 2.0
    Cf = 1.25
    Bf = 1.0
    Dr = 0.8 * m * 9.81 / 2.0
    Cr = 1.25
    Br = 1.0

    x_next_step     = np.zeros(x.shape[0])
    cur_x_next_step = np.zeros(x.shape[0])

    # Extract the value of the states
    delta = u[0,0]
    a     = u[0,1]

    psi = x_glob[3]
    X = x_glob[4]
    Y = x_glob[5]

    vx    = x[0]
    vy    = x[1]
    wz    = x[2]
    epsi  = x[3]
    s     = x[4]
    ey    = x[5]

    # Compute tire split angle
    alpha_f = delta - np.arctan2( vy + lf * wz, vx )
    alpha_r = - np.arctan2( vy - lf * wz , vx)

    # Compute lateral force at front and rear tire
    Fyf = Df * np.sin( Cf * np.arctan(Bf * alpha_f ) )
    Fyr = Dr * np.sin( Cr * np.arctan(Br * alpha_r ) )

    # Propagate the dynamics of deltaT
    x_next_step[0] = a - 1 / m * Fyf * np.sin(delta) + wz*vy
    x_next_step[1] = 1 / m * (Fyf * np.cos(delta) + Fyr) - wz * vx
    x_next_step[2] = 1 / Iz *(lf * Fyf * np.cos(delta) - lr * Fyr)
    x_next_step[3] = wz
    x_next_step[4] = vx * np.cos(psi) - vy * np.sin(psi)
    x_next_step[5] = vx * np.sin(psi)  + vy * np.cos(psi)


    cur,_,_ = getCurvature(X, Y)
    # cur = getCurvature_S(s)
   
    cur_x_next_step[0] = a - 1 / m * Fyf * np.sin(delta) + wz*vy
    cur_x_next_step[1] = 1 / m * (Fyf * np.cos(delta) + Fyr) - wz * vx
    cur_x_next_step[2] = 1 / Iz *(lf * Fyf * np.cos(delta) - lr * Fyr) 
    cur_x_next_step[3] = wz - (vx * np.cos(epsi) - vy * np.sin(epsi)) / (1 - cur * ey) * cur 
    cur_x_next_step[4] = (vx * np.cos(epsi) - vy * np.sin(epsi)) / (1 - cur * ey) 
    cur_x_next_step[5] = vx * np.sin(epsi) + vy * np.cos(epsi)

    return cur_x_next_step, x_next_step


def pid(speed, X, X_inertial):
    """
    Basic controller for speed/steer -> accl./steer vel. This function makes the vehicle complete the lap in constant velocity

        Args:
            speed (float): desired input speed

        Returns:
            U (array - float): [accl, sv]
            accl (float): desired input acceleration
            sv (float): desired input steering velocity
    """
    global oldU
    global prev_ey
    global sum_ey
    global waypoints
    
    position = np.array([X_inertial[4],X_inertial[5]])
    # waypoints = np.loadtxt("IMS_map_waypoints.csv", delimiter=",")
    # waypoints = np.loadtxt("IMS_raceline.csv", delimiter=';', skiprows=3)

    wpts = np.vstack((waypoints[:, 0], waypoints[:, 1])).T
    _,_,_,nearest_point_id,intersected_point = nearest_point_on_trajectory(position, wpts)

    U = np.zeros((1,2),dtype=float)
    
    if(nearest_point_id==0):
        sign = (X_inertial[4]-waypoints[nearest_point_id,0])*(waypoints[nearest_point_id+1,1]-waypoints[nearest_point_id,1]) - (X_inertial[5]-waypoints[nearest_point_id,1])*(waypoints[nearest_point_id+1,0]-waypoints[nearest_point_id,0])
        
    else:
        sign = (X_inertial[4]-waypoints[nearest_point_id,0])*(waypoints[nearest_point_id-1,1]-waypoints[nearest_point_id,1]) - \
                (X_inertial[5]-waypoints[nearest_point_id,1])*(waypoints[nearest_point_id-1,0]-waypoints[nearest_point_id,0])
    
    # dist = ((X_inertial[4]-waypoints[nearest_point_id,0])**2 + (X_inertial[5]-waypoints[nearest_point_id,1])**2)**0.5
    dist = ((X_inertial[4]-intersected_point[0])**2 + (X_inertial[5]-intersected_point[1])**2)**0.5
    
    K_d = -5
    K_i = 0.001
    if(sign<0):
        U[0,0] = -0.9*X[5] - 0.9* X[3]
        # + 1.5*dist
        sum_ey = sum_ey+X[5]
        U[0,0] = U[0,0] + K_d*(prev_ey - X[5]) - K_i*(sum_ey)
        prev_ey = X[5]
    else:
        U[0,0] = -0.9*X[5] - 0.9* X[3] 
        # - 1.5*dist
        sum_ey = sum_ey+X[5]
        U[0,0] =  U[0,0] + K_d*(prev_ey - X[5]) - K_i*(sum_ey)
        prev_ey = X[5]
    
    U[0,1] = 1.5 * (speed - X[0]) 
    # print("lateral error:[%0.4f] | ey:[%0.4f] | epsi:[%0.4f] | U0:[%0.4f]"%(dist,X[5],X[3],U[0,0]))
    # limits on inputs:
    max_heading_rate = 0.5
    max_acceleration = 10
    
    if(abs(U[0,0]) > max_heading_rate):
        if(U[0,0] < 0):
            U[0,0] = -max_heading_rate
        elif(U[0,0]>0):
            U[0,0] = max_heading_rate
            
    if(abs(U[0,0]) > max_acceleration):
        if(U[0,0] < 0):
            U[0,0] = -max_acceleration
        elif(U[0,0]>0):
            U[0,0] = max_acceleration
            
    oldU = U
    return U,np.sign(sign)*dist