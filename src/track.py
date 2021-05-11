import numpy as np
import forcespro
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from structs import *
from scipy.interpolate import CubicSpline
import casadi
class Track:
    def __init__(self,filename):

        self.data = np.genfromtxt(filename, delimiter=',')
        self.n_points = self.data.shape[0]

        self.x_points = self.data[:,0]
        self.y_points = self.data[:,1]
        self.s_points = self.get_s()
        self.track_length = self.s_points[-1]

        self.x_spline = CubicSpline(self.s_points,self.x_points)
        self.y_spline = CubicSpline(self.s_points,self.y_points)
    
    def get_s(self):
        points = np.zeros(self.x_points.shape[0])
        
        for i in range(1,self.x_points.shape[0]):
            points[i] = ((self.x_points[i] - self.x_points[i-1])**2 + (self.y_points[i]-self.y_points[i-1])**2)**0.5 + points[i-1]

        return points

    def get_position(self,s):
        s = s % self.track_length
        return [self.x_spline(s),self.y_spline(s)]

    def get_derivative(self,s):
        s = s % self.track_length
        return [self.x_spline(s,1),self.y_spline(s,1)]

    def get_double_derivative(self,s):
        s = s % self.track_length
        return [self.x_spline(s,2),self.y_spline(s,2)]
    
    def get_proj(self,x,y):
        dists  = (self.x_points - x)**2 + (self.y_points - y)**2
        s_guess = self.s_points[np.argmin(dists)]

        s_opt = s_guess
        s_old = s_guess
        diff = [0,0]
        for i in range(20):
            pos = self.get_position(s_opt)
            d_pos = self.get_derivative(s_opt)
            dd_pos = self.get_double_derivative(s_opt)

            diff[0],diff[1] = pos[0] - x, pos[1] - y

            jacobian = 2 * diff[0] * d_pos[0] + 2 * diff[1] * d_pos[1]
            hessian = 2 * (diff[0] * dd_pos[0] + d_pos[0]**2 + diff[1] * dd_pos[1] + d_pos[1]**2)

            s_opt -= jacobian/hessian
            s_opt = s_opt % self.track_length

            if(abs(s_opt - s_old) < 0.00001):
                return s_opt
            
            s_old = s_opt

        return s_guess%self.track_length


