import forcespro
from config import *
class Solver():
    def __init__(self):
        self.stages = forcespro.MultistagePoblem(no_of_stages)
    
    def set_stages(self,x,u,stage_no):
        self.stages.dims[ i ]['n'] = n_states
