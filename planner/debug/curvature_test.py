# -*- coding: utf-8 -*-
import numpy as np

def getCurvature(X,Y):
    position = np.array([X,Y])
    waypoints = np.loadtxt("IMS_map_waypoints.csv", delimiter=",") 
    # waypoints = np.loadtxt("IMS_raceline.csv", delimiter=';', skiprows=3)

    wpts = np.vstack((waypoints[:, 0], waypoints[:, 1])).T
    _,_,_,nearest_point_id,_ = nearest_point_on_trajectory(position, wpts)
    cur = waypoints[nearest_point_id,4]
    # print("{:.3f}".format(((X-waypoints[nearest_point_id,0])**2 + (Y-waypoints[nearest_point_id,1])**2)**0.5))
    return cur,nearest_point_id,wpts
    
def getCurvature_S(s):
    waypoints = np.loadtxt("IMS_map_waypoints.csv", delimiter=",") 
    # waypoints = np.loadtxt("IMS_raceline.csv", delimiter=';', skiprows=3)
    diff_args = np.argsort(np.abs(waypoints[:,5]-s))
    curv = ((s-waypoints[diff_args[0],5])/(waypoints[diff_args[1],5] - waypoints[diff_args[0],5]))\
            *(waypoints[diff_args[1],4] - waypoints[diff_args[0],4]) + waypoints[diff_args[0],4]
    return curv

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
    # dots = np.sum((point - trajectory[:-1,:]) * diffs[:,:], axis=1)
    dots = np.empty((trajectory.shape[0]-1, ))
    for i in range(dots.shape[0]):
        dots[i] = np.dot((point - trajectory[i, :]), diffs[i, :])
    t = dots / l2s
    t[t<0.0] = 0.0
    t[t>1.0] = 1.0
    # t = np.clip(dots / l2s, 0.0, 1.0)
    projections = trajectory[:-1,:] + (t*diffs.T).T
    # dists = np.linalg.norm(point - projections, axis=1)
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


if __name__== "__main__":
    
    STATES = np.load('PID_states_4_final.npz')
    X_c = STATES['X_c']
    X_g = STATES['X_g']
    U = STATES['U']
    
    for i in range(len(X_g)):
        cur1,nearest_point_id,wpts = getCurvature(X_g[i][4],X_g[i][5])
        cur2 = getCurvature_S(X_c[i][4])
        
        print("Curvatures: 1-> %0.4f | 2-> %0.4f"%(cur1,cur2))