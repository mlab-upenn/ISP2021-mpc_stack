import numpy as np
from config import *
class State:
    def __init__(self,a=[0,0,0,0,0,0,0,0,0,0,0]):
        self.X= a[0]
        self.Y= a[1]
        self.delta = a[2]
        self.v = a[3]
        self.psi = a[4]
        self.psi_dot = a[5]
        self.beta = a[6]
        self.s= a[7]
        self.v_delta = a[8]
        self.a_long = a[9]
        self.v_s = a[10]
        self.vec = np.zeros((11,1))
    def to_vec(self):
        self.vec[:,0] = [self.X,self.Y,self.delta,self.v,self.psi,self.psi_dot,self.beta,self.s,self.v_delta,self.a_long,self.v_s] 
        return self.vec
    def to_list(self):
        return self.to_vec().flatten().tolist()


class Input:
    def __init__(self,a=[0,0,0]):
        self.dv_delta = a[0]
        self.da_long = a[1]
        self.dv_s = a[2]
        self.vec = np.zeros((3,1))

    def set_zero(self):
        self.dv_delta = 0
        self.da_long = 0
        self.dv_s = 0

    def to_vec(self):
        self.vec[:,0] = [self.dv_delta,self.da_long,self.dv_s]
        return self.vec
    
    def to_list(self):
        return self.to_vec().flatten().tolist()

class Opt_vars:
    def __init__(self):
        self.x = State()
        self.u = Input()

class Stage_vars:
    def __init__(self):
        self.Q = np.zeros((n_states,n_states))
        self.R = np.zeros((n_inputs,n_inputs))
        self.q = np.zeros((n_states,1))

        self.A = np.zeros((n_states,n_states))
        self.B = np.zeros((n_states,n_inputs))
        self.g = np.zeros((n_states,1))

        #Polytopic constraints 
        self.P = np.zeros((no_of_polytopic_constraints,n_states))
        # Upper bounds for the polytopic constraints
        self.p_ub = np.zeros((no_of_polytopic_constraints,1))