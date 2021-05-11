import numpy as np
from config import *

class State:
    def __init__(self,a=[0,0,0,0,0,0,0,0,0,0]):
        self.X= a[0]
        self.Y= a[1]
        self.delta = a[2]
        self.v = a[3]
        self.psi = a[4]
        self.psi_dot = a[5]
        self.beta = a[6]
        self.s= a[7]
        self.v_s = a[8]
        self.v_delta = a[9]
        self.vec = np.zeros((n_states,1))
    def to_vec(self):
        self.vec[:,0] = [self.X,self.Y,self.delta,self.v,self.psi,self.psi_dot,self.beta,self.s,self.v_s,self.v_delta] 
        return self.vec
    def to_list(self):
        return self.to_vec().flatten().tolist()
    def __mul__(self,num):
        return State([num * self.X,num * self.Y,num * self.delta,num * self.v,num * self.psi,num * self.psi_dot,num * self.beta,num * self.s,num * self.v_s,num * self.v_delta])
    def __add__(self,state):
        return State([state.X + self.X,state.Y + self.Y,state.delta + self.delta,state.v + self.v,state.psi + self.psi,state.psi_dot + self.psi_dot,state.beta + self.beta,state.s + self.s,state.v_s + self.v_s, state.v_delta + self.v_delta])


class Input:
    def __init__(self,a=[0,0,0]):
        self.dv_delta = a[0]
        self.a_long = a[1]
        self.dv_s = a[2]
        self.vec = np.zeros((n_inputs,1))

    def set_zero(self):
        self.dv_delta = 0
        self.a_long = 0
        self.dv_s = 0

    def to_vec(self):
        self.vec[:,0] = [self.dv_delta,self.a_long,self.dv_s]
        return self.vec
    
    def to_list(self):
        return self.to_vec().flatten().tolist()
    def __mul__(self,num):
        return Input([num * self.dv_delta, num * self.a_long, num * self.dv_s])
    def __add__(self,inp):
        return Input([inp.dv_delta + self.dv_delta, inp.a_long + self.a_long, inp.dv_s + self.dv_s])

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