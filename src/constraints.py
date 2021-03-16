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
        
        #C * [state_vector] < upper_bound
        C = np.zeros((no_of_polytopic_constraints,n_states))
        upper_bound = np.zeros((no_of_polytopic_constraints,1))
        
        C[0,0:2] = [-d_pos_y, d_pos_x]
        upper_bound[0,0] = d_pos_x * outer_point_y - d_pos_y * outer_point_x

        C[0,0:2] = [d_pos_y, -d_pos_x]
        upper_bound[1,0] = - (d_pos_x * inner_point_y - d_pos_y * inner_point_x)
        
        return C,upper_bound
    
    def vis_track_constraints(self,x,y,track):
        
        s = track.get_proj(x,y)
        pos_x, pos_y = track.get_position(s)
        d_pos_x, d_pos_y = track.get_derivative(s)

        outer_point_x = pos_x + track_scale_factor * (-d_pos_y)
        outer_point_y = pos_y + track_scale_factor * (d_pos_x)
        inner_point_x = pos_x - track_scale_factor * (-d_pos_y)
        inner_point_y = pos_y - track_scale_factor * (d_pos_x)

        return d_pos_x,d_pos_y,outer_point_x,outer_point_y,inner_point_x,inner_point_y

    def get_constraints(self,x,track):
        return self.get_track_constraints(x,track)