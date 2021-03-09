
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

        self.Q_cross = 0.1
        self.Q_cross_N = 1
        self.Q_lag = 500
        self.Q_lag_N = 500
        self.Q_v_s = 0.02 

        self.R_dv_delta = 0.00001
        self.R_da_long = 0.00001
        self.R_dv_s = 0.00001
        
        self.X_u = 1000
        self.Y_u = 1000
        self.delta_u = 3.14/2
        self.v_u = 7
        self.psi_u = 2 * 3.14
        self.psi_dot_u =  3.2
        self.beta_u = 3.14/4
        self.s_u = 1000
        self.v_delta_u = 3.2
        self.a_long_u = 7.51
        self.v_s_u = 7

        self.X_l = -1000
        self.Y_l = -1000
        self.delta_l = -3.14/2
        self.v_l = -7
        self.psi_l = 0
        self.psi_dot_l = -3.2
        self.beta_l = 3.14/4
        self.s_l = 0
        self.v_delta_l = -3.2
        self.a_long_l = -7.51
        self.v_s_l = 0