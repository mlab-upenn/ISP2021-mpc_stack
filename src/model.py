import numpy as np
from config import *
from structs import *
from params import Params
from scipy.linalg import expm

class Model:
    def __init__(self):
        self.param = Params()
        self.c_psi_dot = self.param.m * self.param.mu / (self.param.I_z * (self.param.l_r + self.param.l_f))

        self.c1_d_psi_double_dot = (self.param.m * self.param.mu / (self.param.I_z * (self.param.l_r + self.param.l_f)))
        self.c1_beta_dot= self.param.mu/(self.param.l_f + self.param.l_r)

        self.l_f_mul_C_sf = self.param.l_f * self.param.C_sf
        self.l_r_mul_C_sr = self.param.l_r * self.param.C_sr
        self.g_mul_l_f = self.param.g * self.param.l_f
        self.g_mul_l_r = self.param.g * self.param.l_r
        self.C_sf_mul_h_cg = self.param.C_sf * self.param.h_cg
        self.C_sr_mul_h_cg = self.param.C_sr * self.param.h_cg

    
    # f is the x_dot at a operating point
    def get_f(self,x,u):
        f = np.zeros((n_states,1))

        if(x.v == 0):
            x.v = 0.01
        
        # X_dot = v * cos(beta + psi)
        f[0,0] = x.v * np.cos(x.beta + x.psi)
        # Y_dot = v * sin(beta + psi)
        f[1,0] = x.v * np.sin(x.beta + x.psi)
        # delta_dot = v_delta
        f[2,0] = x.v_delta
        # v_dot = a_long
        f[3,0] = u.a_long
        # psi_dot = psi_dot
        f[4,0] = x.psi_dot
        # psi_double_dot = *Refer the documentation*
        f[5,0] = (self.param.m * self.param.mu / (self.param.I_z * (self.param.l_r + self.param.l_f))) * \
        (self.l_f_mul_C_sf * (self.g_mul_l_r - u.a_long * self.param.h_cg) * x.delta + \
        (self.l_r_mul_C_sr * (self.g_mul_l_f + u.a_long * self.param.h_cg) - \
        self.l_f_mul_C_sf * (self.g_mul_l_r - u.a_long * self.param.h_cg)) * x.beta - \
        (self.param.l_f**2 * self.param.C_sf * (self.g_mul_l_r - u.a_long * self.param.h_cg) + \
        self.param.l_r**2 * self.param.C_sr * (self.g_mul_l_f + u.a_long * self.param.h_cg))*(x.psi_dot/x.v))
        # beta_dot = **Refer the documentation*
        f[6,0] = (self.param.mu / (x.v * (self.param.l_r + self.param.l_f))) * \
        (self.param.C_sf * (self.g_mul_l_r - u.a_long * self.param.h_cg) * x.delta - \
        (self.param.C_sr * (self.g_mul_l_f + u.a_long * self.param.h_cg) + \
        self.param.C_sf * (self.g_mul_l_r - u.a_long * self.param.h_cg)) * x.beta + \
        (self.param.C_sr * (self.g_mul_l_f + u.a_long * self.param.h_cg) * self.param.l_r - \
        self.param.C_sf * (self.g_mul_l_r - u.a_long * self.param.h_cg) * self.param.l_f) * (x.psi_dot/x.v)) - \
        x.psi_dot
        # s_dot = v_s
        f[7,0] = x.v_s
        # v_s_dot = dv_s
        f[8,0] = u.dv_s
        # v_delta_dot = dv_delta
        f[9,0] = u.dv_delta
        return f

    # Linearize the differential equations to get the A, B and g matrices
    def get_jacobian(self,x,u):
        
        # Dont let the velocity to be small
        x.v = max(0.001,x.v)
        A = np.zeros((n_states,n_states))
        B = np.zeros((n_states,n_inputs))
        # X_dot
        A[0] = [0,0,0,np.cos(x.beta + x.psi), -x.v * np.sin(x.beta + x.psi),0,-x.v * np.sin(x.beta + x.psi),0,0,0]
        # Y_dot
        A[1] = [0,0,0,np.sin(x.beta + x.psi),  x.v * np.cos(x.beta + x.psi),0, x.v * np.cos(x.beta + x.psi),0,0,0]
        # delta_dot
        A[2] = [0,0,0,0,0,0,0,0,0,1]
        # v_dot
        B[3] = [0,1,0]
        # psi_dot
        A[4] = [0,0,0,0,0,1,0,0,0,0]
        
        d_psi_double_dot_a_long = self.c1_d_psi_double_dot * \
        (-self.l_f_mul_C_sf * self.param.h_cg * x.delta + self.l_r_mul_C_sr * self.param.h_cg * x.beta + \
        self.l_f_mul_C_sf * self.param.h_cg * x.beta + \
        (self.param.l_f**2 * self.C_sf_mul_h_cg + self.param.l_r**2 * self.C_sr_mul_h_cg)*(x.psi_dot/x.v))

        d_psi_double_dot_delta = self.c1_d_psi_double_dot * \
        (self.l_f_mul_C_sf * (self.g_mul_l_r - u.a_long * self.param.h_cg))

        d_psi_double_dot_beta = self.c1_d_psi_double_dot * \
        (self.l_r_mul_C_sr * (self.g_mul_l_f + u.a_long * self.param.h_cg) - \
        self.l_f_mul_C_sf * (self.g_mul_l_r - u.a_long * self.param.h_cg))

        d_psi_double_dot_psi_dot = (-1/x.v) * self.c1_d_psi_double_dot * \
        (self.param.l_f**2 * self.param.C_sf * (self.g_mul_l_r - u.a_long * self.param.h_cg) + \
        self.param.l_r**2 * self.param.C_sr *(self.g_mul_l_f + u.a_long * self.param.h_cg))

        d_psi_double_dot_v = (x.psi_dot/x.v**2) * self.c1_d_psi_double_dot * \
        (self.param.l_f**2 * self.param.C_sf * (self.g_mul_l_r - u.a_long * self.param.h_cg) + \
        self.param.l_r**2 * self.param.C_sr *(self.g_mul_l_f + u.a_long * self.param.h_cg))

        # psi_double_dot
        A[5] = [0,0,d_psi_double_dot_delta, d_psi_double_dot_v, 0,d_psi_double_dot_psi_dot, d_psi_double_dot_beta, 0,0,0]
        B[5] = [0,d_psi_double_dot_a_long,0]
        
        c = self.param.C_sf * (self.g_mul_l_r - u.a_long * self.param.h_cg)*x.delta - \
        (self.param.C_sr * (self.g_mul_l_f + u.a_long * self.param.h_cg) + \
        self.param.C_sf * (self.g_mul_l_r - u.a_long * self.param.h_cg) ) * x.beta

        d = (self.param.C_sr * (self.g_mul_l_f + u.a_long * self.param.h_cg) * self.param.l_r -\
        self.param.C_sf * (self.g_mul_l_r - u.a_long * self.param.h_cg) * self.param.l_f) 

        d_beta_dot_v = (-self.c1_beta_dot/x.v**2) * (c + (2 * d * x.psi_dot/x.v))

        d_beta_dot_a_long = (self.c1_beta_dot/x.v) * ( -self.C_sf_mul_h_cg * x.delta - self.C_sr_mul_h_cg * x.beta +\
        self.C_sf_mul_h_cg * x.beta + \
        (self.C_sr_mul_h_cg * self.param.l_r + self.C_sf_mul_h_cg * self.param.l_f)*(x.psi_dot/x.v))

        d_beta_dot_delta = (self.c1_beta_dot/x.v) * (self.param.C_sf * (self.g_mul_l_r - u.a_long * self.param.h_cg))

        d_beta_dot_psi_dot = (self.c1_beta_dot/x.v) * (d/x.v) - 1
        d_beta_dot_beta  =(self.c1_beta_dot/x.v) * (-self.param.C_sr * (self.g_mul_l_f + u.a_long * self.param.h_cg) -\
        self.param.C_sf * (self.g_mul_l_r - u.a_long * self.param.h_cg))

        # beta_dot
        A[6] = [0,0, d_beta_dot_delta, d_beta_dot_v, 0, d_beta_dot_psi_dot, d_beta_dot_beta, 0, 0,0]
        B[6] = [0,d_beta_dot_a_long,0]
        # s_dot
        A[7] = [0,0,0,0,0,0,0,0,1,0]

        # v_s_dot
        B[8] = [0,0,1]
        # v_delta_dot
        B[9] = [1,0,0]

        g = self.get_f(x,u) - (A @ x.to_vec() + B @ u.to_vec())

        return A,B,g

    # Discretize the continuous model. Refer wikipedia for the trick
    def discretize(self,A,B,g,x,track_length):
        M = np.zeros((n_states + n_inputs + 1,n_states + n_inputs + 1))
        M[0:n_states,0:n_states] = A
        M[0:n_states,n_states:n_states+n_inputs] = B
        M[0:n_states,[n_states+n_inputs]] = g
        
        M_exp = expm(M * sampling_time)

        A_d = M_exp[0:n_states,0:n_states]
        
        B_d = M_exp[0:n_states,n_states:n_states+n_inputs]
        g_d = M_exp[0:n_states,[n_states+n_inputs]]

        return A_d, B_d, g_d
    
    # Get the linearized model
    def get_lin_model(self,x,u,track_length):
        A,B,g = self.get_jacobian(x,u)
        A_d,B_d,g_d = self.discretize(A,B,g,x,track_length)

        return A_d,B_d,g_d






        
