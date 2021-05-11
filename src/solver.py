from forcespro import *
from config import *
import numpy as np
from params import *
import get_userid
# import warnings
# warnings.simplefilter("error")
class Solver():
    def __init__(self):
        self.stages = MultistageProblem(no_of_stages)
        self.param = Params()
    
    def gen_solver(self):
    
        for i in range(no_of_stages):
            # Stage variable z = [u;x]
            self.stages.dims[i]['n'] = n_inputs + n_states 	# number of stage variables
            self.stages.dims[i]['r'] = n_states		        # number of equality constraints
            self.stages.dims[i]['l'] = n_inputs + n_states 	# number of lower bounds
            self.stages.dims[i]['u'] = n_inputs + n_states 	# number of upper bounds
            self.stages.dims[i]['p'] = no_of_polytopic_constraints # number of polytopic constraints
            
            self.stages.newParam('H{:02d}'.format(i+1), [i+1], 'cost.H')
            self.stages.newParam('f{:02d}'.format(i+1), [i+1], 'cost.f')

            # lower bound acts on these indices
            self.stages.ineq[i]['b']['lbidx'] = list(range(1, n_inputs + n_states + 1))
            # lower bound for this stage variable
            self.stages.ineq[i]['b']['lb'] = np.concatenate((self.param.get_input_lb(), self.param.get_state_lb()), 0)

            # upper bound acts on these indices
            self.stages.ineq[i]['b']['ubidx'] = list(range(1, n_inputs + n_states + 1))
            # upper bound for this stage variable
            self.stages.ineq[i]['b']['ub'] = np.concatenate((self.param.get_input_ub(), self.param.get_state_ub()), 0)
            
            self.stages.newParam('A{:02d}'.format(i+1), [i+1], 'ineq.p.A')
            self.stages.newParam('b{:02d}'.format(i+1), [i+1], 'ineq.p.b')
            
            if i < no_of_stages - 1:
                self.stages.newParam('C{:02d}'.format(i+1), [i+1], 'eq.C')
            if i > 0:
                self.stages.newParam('c{:02d}'.format(i+1), [i+1], 'eq.c')
            
            self.stages.newParam('D{:02d}'.format(i+1), [i+1], 'eq.D')
        
        self.stages.newParam('minusA_times_x0', [1], 'eq.c')  # RHS of first eq. constr. is a parameter: z1=-A*x0

        # define output of the solver
        self.stages.newOutput('u0', list(range(1,no_of_stages+1)), list(range(1, n_inputs + n_states + 1)))

        # solver settings
        self.stages.codeoptions['name'] = 'mpcc'
        self.stages.codeoptions['printlevel'] = 1

        self.stages.generateCode(get_userid.userid)
    
    def solve(self,opt_stages,x0):

        import mpcc_py

        problem = mpcc_py.mpcc_params

        for i in range(no_of_stages):
            problem['H{:02d}'.format(i+1)] = np.vstack((np.hstack((opt_stages[i+1].R,np.zeros((n_inputs,n_states)))),np.hstack((np.zeros((n_states,n_inputs)),opt_stages[i+1].Q))))
            problem['f{:02d}'.format(i+1)] = np.vstack((np.zeros((n_inputs,1)),opt_stages[i+1].q))
            
            problem['A{:02d}'.format(i+1)] = np.hstack((np.zeros((no_of_polytopic_constraints,n_inputs)),opt_stages[i+1].P))
            problem['b{:02d}'.format(i+1)] = opt_stages[i+1].p_ub

            if(i<no_of_stages-1):
                problem['C{:02d}'.format(i+1)] = np.hstack((np.zeros((n_states,n_inputs)),opt_stages[i+1].A))
            if(i>0):
                problem['c{:02d}'.format(i+1)] = -opt_stages[i+1].g
            problem['D{:02d}'.format(i+1)] = np.hstack((opt_stages[i+1].B,-np.eye(n_states)))
        
        try:
            # print("A  ",opt_stages[0].A)
            # print("x  ",x0.to_vec())
            problem['minusA_times_x0'] = -(opt_stages[0].A @ x0.to_vec() + opt_stages[0].g)
        except RuntimeWarning:
            print("Invalid value")
            print("A  ",opt_stages[0].A)
            print("x  ",x0.to_vec())
            print("g  ",opt_stages[0].g)


        [solverout, exitflag, info] = mpcc_py.mpcc_solve(problem)
        if (exitflag == 1):
            pass
            # print(solverout['u0'])
            # print('Problem solved in %5.3f milliseconds (%d iterations).' % (1000.0 * info.solvetime, info.it))
        else:
            print("fail")
        return solverout['u0'],exitflag
