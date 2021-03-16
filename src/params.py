import numpy as np
from config import *
class Params:
    def __init__(self):
        self.mu = 1.0489        #Friction coefficient
        self.m = 3.47           #Mass of the car
        self.I_z = 0.04712      #Moment of inertia
        self.l_r = 0.17145      #Length from rear axle to Center of gravity
        self.l_f = 0.15875      #Length from front axle to Center of gravity
        self.C_sf = 4.718       #Cornering stiffness front
        self.C_sr = 5.4562      #Cornering stiffness rear
        self.h_cg = 0.074       #Height of center of gravity
        self.g = 9.81           #Gravity

        self.init_vel = 1
        self.init_psi = 0
        
        self.Q_cross = 1
        self.Q_cross_N = 1
        self.Q_lag = 5
        self.Q_lag_N = 5
        self.Q_v_s = 0.02

        # self.R_dv_delta = 0.00001
        # self.R_da_long = 0.00001
        # self.R_dv_s = 0.00001
        self.R_dv_delta = 0.00001
        self.R_da_long = 0.1
        self.R_dv_s = 0.00001

        self.X_ub = 100
        self.Y_ub = 100
        self.delta_ub = 0.34
        self.v_ub = 11
        self.psi_ub = 3.14
        self.psi_dot_ub =  3.2
        self.beta_ub = 3.14/90
        self.s_ub = 500
        self.v_delta_ub = 3.2
        self.a_long_ub = 7.51
        self.v_s_ub = 7

        self.X_lb = -100
        self.Y_lb = -100
        self.delta_lb = -0.34
        self.v_lb = -5
        self.psi_lb = -3.14
        self.psi_dot_lb = -3.2
        self.beta_lb = -3.14/90
        self.s_lb = 0
        self.v_delta_lb = -3.2
        self.a_long_lb = -7.51
        self.v_s_lb = 0

        self.dv_delta_ub = 5
        self.da_long_ub = 10
        self.dv_s_ub = 100

        self.dv_delta_lb = -5
        self.da_long_lb = -10
        self.dv_s_lb = -100

        # self.X_ub = 100
        # self.Y_ub = 100
        # self.delta_ub = np.pi/4
        # self.v_ub = 5
        # self.psi_ub = 2 * 3.14
        # self.psi_dot_ub =  1000
        # self.beta_ub = 3.14
        # self.s_ub = 10000
        # self.v_delta_ub = 10000
        # self.a_long_ub = 10000
        # self.v_s_ub = 10000

        # self.X_lb = -100
        # self.Y_lb = -100
        # self.delta_lb = -np.pi/4
        # self.v_lb = -5
        # self.psi_lb = 0
        # self.psi_dot_lb = -10000
        # self.beta_lb = 0
        # self.s_lb = 0
        # self.v_delta_lb = -10000
        # self.a_long_lb = -10000
        # self.v_s_lb = -10000

        # self.dv_delta_ub = 10000
        # self.da_long_ub = 10000
        # self.dv_s_ub = 10000

        # self.dv_delta_lb = -10000
        # self.da_long_lb = -10000
        # self.dv_s_lb = -10000
    
    def get_state_ub(self):
        ub = np.zeros((n_states))
        ub[0] = self.X_ub
        ub[1] = self.Y_ub
        ub[2] = self.delta_ub
        ub[3] = self.v_ub
        ub[4] = self.psi_ub
        ub[5] = self.psi_dot_ub
        ub[6] = self.beta_ub
        ub[7] = self.s_ub
        ub[8] = self.v_delta_ub
        ub[9] = self.a_long_ub
        ub[10] = self.v_s_ub
        return ub
    
    def get_state_lb(self):
        lb = np.zeros((n_states))
        lb[0] = self.X_lb
        lb[1] = self.Y_lb
        lb[2] = self.delta_lb
        lb[3] = self.v_lb
        lb[4] = self.psi_lb
        lb[5] = self.psi_dot_lb
        lb[6] = self.beta_lb
        lb[7] = self.s_lb
        lb[8] = self.v_delta_lb
        lb[9] = self.a_long_lb
        lb[10] = self.v_s_lb
        return lb
    
    def get_input_ub(self):
        ub = np.zeros((n_inputs))
        ub[0] = self.dv_delta_ub
        ub[1] = self.da_long_ub
        ub[2] = self.dv_s_ub
        return ub

    def get_input_lb(self):
        lb = np.zeros((n_inputs))
        lb[0] = self.dv_delta_lb
        lb[1] = self.da_long_lb
        lb[2] = self.dv_s_lb
        return lb
