import numpy as np
from params import *
from track import *
from config import *
class Constraints:
    def __init__(self):
        self.param = Params()
    
    def get_track_constraints(self,x,track):
        
        pos_x, pos_y = track.get_position(x.s)
        d_pos_x, d_pos_y = track.get_derivative(x.s)

        outer_point_x = pos_x + track_scale_factor * (-d_pos_y)
        outer_point_y = pos_y + track_scale_factor * (d_pos_x)
        inner_point_x = pos_x - track_scale_factor * (-d_pos_y)
        inner_point_y = pos_y - track_scale_factor * (d_pos_x)
        C = np.zeros((2,1))

        # lower_bound < C * [x] < upper_bound
        #                   [y] 
        C = [-d_pos_y, d_pos_x]
        lower_bound = d_pos_x * inner_point_y - d_pos_y * inner_point_x
        upper_bound = d_pos_x * outer_point_y - d_pos_y * outer_point_x
        
        return C,lower_bound,upper_bound


    def get_constraints(self,x,track):
        pass