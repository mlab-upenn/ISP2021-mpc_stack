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
from scipy.integrate import solve_ivp
class MPCC:
    def __init__(self):
        self.param = Params()
        self.init_guess = [Opt_vars() for i in range(no_of_stages+1)]
        self.stages = [Stage_vars() for i in range(no_of_stages+1)]
        self.track = Track('/home/msnaga/f1_10_ws/src/mpcc/data/Silverstone_map_waypoints.csv')
        self.cost = Cost()
        self.model = Model()
        self.constraint = Constraints()
        self.solver = Solver()
        self.initial_guess = False

    def gen_inital_guess(self, x0):
        self.init_guess[0].x = x0
        for i in range(1,no_of_stages+1):
            self.init_guess[i].x.s = self.init_guess[i-1].x.s + sampling_time * self.param.init_vel

            self.init_guess[i].x.X,self.init_guess[i].x.Y = self.track.get_position(self.init_guess[i].x.s)
            dx, dy = self.track.get_derivative(self.init_guess[i].x.s)
            self.init_guess[i].x.psi = np.arctan2(dy,dx)
            self.init_guess[i].x.v = self.param.init_vel
            self.init_guess[i].x.v_s = self.param.init_vel
        self.initial_guess = True

    def set_stage(self,x,u,stage_curr):
        self.stages[stage_curr].Q,self.stages[stage_curr].q,self.stages[stage_curr].R = self.cost.get_cost(x,self.track,stage_curr)
        self.stages[stage_curr].A,self.stages[stage_curr].B,self.stages[stage_curr].g = self.model.get_lin_model(x,u)
        self.stages[stage_curr].P,self.stages[stage_curr].p_ub = self.constraint.get_constraints(x,self.track)            
    
    def set_mpcc(self):
        for i in range(no_of_stages+1):
            self.set_stage(self.init_guess[i].x,self.init_guess[i].u,i)

    def update_initial_guess(self,data):
        # TODO use the output of solver
        data = data.reshape(-1,n_states_inputs)
        
        for i in range(1,no_of_stages+1):
            self.init_guess[i-1] = self.init_guess[i]
            # self.init_guess[i-1].u = Input(data[i,:3].tolist())
            # self.init_guess[i-1].x = State(data[i,3:].tolist())        

        def integrate(t,x,u0,u1,u2):
            return self.model.get_f(State(x),Input([u0,u1,u2])).flatten().tolist()
        
        out = solve_ivp(integrate,[0,sampling_time],self.init_guess[-2].x.to_list(),args=(self.init_guess[-2].u.to_list()))
        self.init_guess[-1].x = State(out.y[:,-1].tolist())
        self.init_guess[-1].u.set_zero()


    def mpcc_run(self,x):
        # self.solver.gen_solver()
        # TODO project onto spline
        x.s = self.track.get_proj(x.X,x.Y)
        print(x.s)
        self.init_guess[0].x = x
        self.init_guess[0].u.set_zero()

        if(self.initial_guess == False):
            self.gen_inital_guess(x)
        self.set_mpcc()
        data,status = self.solver.solve(self.stages,x)
        self.update_initial_guess(data)
        # Update inital guesses
        return data,status

if __name__== "__main__":
    s = MPCC()
    x = State()
    x.v = 0.0001
    s.mpcc_run(x)
