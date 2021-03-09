import numpy as np
from structs import *
from track import *
from config import *
from params import *

# TODO Penalize yaw rate and others
class Cost:
    def __init__(self):
        self.param = Params()
        pass
    
    def get_track_error_cost(self,x,track,stage_curr):
    
        x_curr,y_curr = x.X,x.Y
        x_ref,y_ref = track.get_position(x.s)
        dx_ref,dy_ref = track.get_derivative(x.s)
        ddx_ref,ddy_ref = track.get_double_derivative(x.s)
        
        # Slope of the curve
        phi_ref = np.arctan2(dy_ref,dx_ref)
        cos_phi_ref = np.cos(phi_ref)
        sin_phi_ref = np.sin(phi_ref)

        # Curvature of the curve
        dphi_ref = (dx_ref * ddy_ref - dy_ref * ddx_ref) / ((dx_ref**2 + dy_ref**2)**(3/2))

        # Cross track error and its partials with the states X,Y,s
        cross_track_err = sin_phi_ref * (x_curr - x_ref) - cos_phi_ref * (y_curr - y_ref)
        d_cross_track_err_X = sin_phi_ref 
        d_cross_track_err_Y = -cos_phi_ref
        d_cross_track_err_s = dphi_ref * cos_phi_ref * (x_curr - x_ref) - sin_phi_ref * dx_ref + \
        dphi_ref * sin_phi_ref * (y_curr - y_ref) + cos_phi_ref * dy_ref 

        # Lag error and its partials with the states X,Y,s
        lag_err = -cos_phi_ref * (x_curr - x_ref) - sin_phi_ref * (y_curr - y_ref)
        d_lag_err_X = -cos_phi_ref
        d_lag_err_Y = -sin_phi_ref
        d_lag_err_s = dphi_ref * sin_phi_ref * (x_curr - x_ref) + cos_phi_ref * dx_ref -\
        dphi_ref * cos_phi_ref * (y_curr - y_ref) + sin_phi_ref * dy_ref

        d_e_cross = np.zeros((1,n_states))
        d_e_cross[0,0] = d_cross_track_err_X
        d_e_cross[0,1] = d_cross_track_err_Y
        d_e_cross[0,7] = d_cross_track_err_s

        d_e_lag = np.zeros((1,n_states))
        d_e_lag[0,0] = d_lag_err_X
        d_e_lag[0,1] = d_lag_err_Y
        d_e_lag[0,7] = d_lag_err_s

        # Linearized Error cost function
        # x.T * (d_e.T * d_e) * x + 2 (e - d_e * state) * d_e * x , state is operating point about which linearization is performed
        # x.T * (Q_cross + Q_lag) * x + (q_cross + q_lag) * x

        # Cost based on the terminal stage or not
        if stage_curr < no_of_stages:
            Q_cross_cost = self.param.Q_cross
            Q_lag_cost = self.param.Q_lag
        else:
            Q_cross_cost = self.param.Q_cross_N
            Q_lag_cost = self.param.Q_lag_N

        Q = Q_cross_cost * d_e_cross.T @ d_e_cross + Q_lag_cost * d_e_lag.T @ d_e_lag
        q = Q_cross_cost * 2 * (cross_track_err - d_e_cross @ x.to_vec) * d_e_cross + Q_lag_cost * 2 * (lag_err - d_e_lag @ x.to_vec) * d_e_lag
        return Q,q

    def get_input_cost(self):
        # TODO cost of real inputs
        # Q = np.zeros((n_states,n_states))
        R = np.zeros((n_inputs,n_inputs))
        R[0,0] = self.param.R_dv_delta
        R[1,1] = self.param.R_da_long
        R[2,2] = self.param.R_dv_s
        return R

    def get_progress_cost(self):
        q = np.zeros((1,n_states))
        q[0,10] = -self.param.Q_v_s
        return q

    def get_cost(self,x,track,stage_curr):
        Q_track, q_track = self.get_track_error_cost(x,track,stage_curr)
        R = self.get_input_cost()
        q_progress = self.get_progress_cost()

        Q_total = Q_track
        q_total = q_track + q_progress

        return Q_total, q_total, R