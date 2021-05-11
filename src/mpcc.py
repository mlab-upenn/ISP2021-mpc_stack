import numpy as np
from solver import *
from config import *
from params import *
from structs import *
from track import *
from cost import *
from model import *
from constraints import *
from solver import *

class MPCC:
    def __init__(self,path):
        self.param = Params()                                           # Params object
        self.init_guess = [Opt_vars() for i in range(no_of_stages+1)]   # Optimization variables for the solver
        self.stages = [Stage_vars() for i in range(no_of_stages+1)]     # Stages matrices
        self.track = Track(path)                                        # Track object
        self.cost = Cost()                                              # Cost object   
        self.model = Model()                                            # Model object
        self.constraint = Constraints()                                 # Constraints object
        self.solver = Solver()                                          # Solver object
        self.initial_guess = False                                      # Flag to see if it is the first solver iteration
        self.iter = 0                                                   # COunt the number of iterations
    
    # Function to generate the inital guess for the solver
    def gen_inital_guess(self, x0):
        self.init_guess[0].x = x0
        for i in range(1,no_of_stages+1):
            # Use the progress parameter dynamics to update 
            self.init_guess[i].x.s = self.init_guess[i-1].x.s + sampling_time * self.param.s_init_vel

            self.init_guess[i].x.X,self.init_guess[i].x.Y = self.track.get_position(self.init_guess[i].x.s)
            dx, dy = self.track.get_derivative(self.init_guess[i].x.s)
            self.init_guess[i].x.psi = np.arctan2(dy,dx)
            self.init_guess[i].x.v = self.param.init_vel
            self.init_guess[i].x.v_s = self.param.s_init_vel
        
        # Set the flag to true
        self.initial_guess = True

    # Function to set the stage matrices for the solver
    def set_stage(self,x,u,stage_curr): 
        self.stages[stage_curr].Q,self.stages[stage_curr].q,self.stages[stage_curr].R = self.cost.get_cost(x,self.track,stage_curr)
        self.stages[stage_curr].A,self.stages[stage_curr].B,self.stages[stage_curr].g = self.model.get_lin_model(x,u,self.track.track_length)
        self.stages[stage_curr].P,self.stages[stage_curr].p_ub = self.constraint.get_constraints(x,self.track)
        
    # Function to set up the MPCC Problem
    def set_mpcc(self):
        for i in range(no_of_stages+1):
            self.set_stage(self.init_guess[i].x,self.init_guess[i].u,i)

    # Wrap the yaw and the progress parameter
    def wrap_inital_guess(self):
        for i in range(0,no_of_stages+1):
            self.init_guess[i].x.psi = np.arctan2(np.sin(self.init_guess[i].x.psi), np.cos(self.init_guess[i].x.psi))
            self.init_guess[i].x.s = self.init_guess[i].x.s % self.track.track_length

    # Update the initial guesses after every iteration of the solver
    def update_initial_guess(self,data):
        
        data = data.reshape(-1,n_states_inputs)
        
        # Shift the array by one behind index
        for i in range(1,no_of_stages+1):           
            self.init_guess[i-1].x = self.init_guess[i].x * mixing + State(data[i-1,3:].tolist()) * (1-mixing)
            self.init_guess[i-1].u = self.init_guess[i].u * mixing + Input(data[i-1,:3].tolist()) * (1-mixing)
        
        # Copy the last element from the penultimate stage
        self.init_guess[-1].x = self.init_guess[-2].x
        self.init_guess[-1].u = self.init_guess[-2].u
        

    def mpcc_run(self,x):
        # Uncomment the below line to generate the solver interface file
        # self.solver.gen_solver()

        self.iter+=1

        # Use the inital guess to update the optimization variables
        self.init_guess[0].x = x
        self.init_guess[0].u.set_zero()

        
        self.wrap_inital_guess()

        if(self.initial_guess == False):
            self.gen_inital_guess(x)
        
        # Set and call the solver
        self.set_mpcc()
        data,status = self.solver.solve(self.stages,x)
    
        # Update inital guesses
        self.update_initial_guess(data)

        return data,status

if __name__== "__main__":
    s = MPCC('/home/msnaga/f1_10_ws/src/mpcc/data/IMS_centerline.csv')
    x = State()
    x.v = 0.0001
    x.psi = np.pi/4
    x.v_s = 1
    s.mpcc_run(x)
