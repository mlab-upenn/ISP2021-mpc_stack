import numpy as np
class State:
    def __init__(self):
        self.X=0
        self.Y=0
        self.delta = 0
        self.v = 0
        self.psi = 0
        self.psi_dot = 0
        self.beta = 0
        self.s=0
        self.v_delta = 0
        self.a_long = 0
        self.v_s = 0
        self.vec = np.zeros((11,1))
    def to_vec(self):
        self.vec[:,0] = [self.X,self.Y,self.delta,self.v,self.psi,self.psi_dot,self.beta,self.s,self.v_delta,self.a_long,self.v_s] 
        return self.vec


class Input:
    def __init__(self):
        self.dv_delta = 0
        self.da_long = 0
        self.dv_s = 0
        self.vec = np.zeros((3,1))
    def to_vec(self):
        self.vec[:,0] = [self.dv_delta,self.da_long,self.dv_s]
        return self.vec

class Point:
    def __init__(self):
        self.x = 0
        self.y = 0 