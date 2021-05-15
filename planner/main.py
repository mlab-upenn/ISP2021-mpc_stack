import numpy as np
import osqp
import gym
import time
from numba import njit
import yaml
from argparse import Namespace
from dataclasses import dataclass, field
import pdb
import math

import cProfile, pstats, io
from pstats import SortKey


import sys
sys.path.append('planner/')
# sys.path.append('fnc/controller')
# sys.path.append('fnc')

# from planner.initControllerParameters import initMPCParams, initLMPCParams
# from planner.PredictiveControllers import MPC, LMPC, MPCParams
# from planner.PredictiveModel import PredictiveModel

from initControllerParameters import initLMPCParams
from PredictiveControllers import LMPC, MPCParams
from PredictiveModel import PredictiveModel

env_name = 'f110_gym:f110-v0'
if env_name in gym.envs.registry.env_specs:
    del gym.envs.registration.registry.env_specs[env_name] 


# with cProfile.Profile() as pr:
if __name__== "__main__":
    # instantiating the environment
    with open('config_IMS_map.yaml') as file:
        conf_dict = yaml.load(file, Loader=yaml.FullLoader)
    conf = Namespace(**conf_dict)
    env = gym.make('f110_gym:f110-v0', map=conf.map_path, map_ext=conf.map_ext, num_agents=1)
    obs, step_reward, done, info = env.reset(np.array([[conf.sx, conf.sy, conf.stheta]]))

    # simulation loop
    simulatorTotalRuntime = 0
    noOfLapsCompleted = 0
    N = 14 # the finite horizon

    X_curv_points = []
    X_glob_points = []
    U_points = []
    
    PIDflag = True
    loadedFile = True
    
    wait_time = 0
    count = 0
    
    # initialization of LMPC parameters
    numSS_it, numSS_Points, Laps, TimeLMPC, QterminalSlack, lmpcParameters = initLMPCParams(N)
    
    currentLapTime = 0
    lapTimes = []
    
    X_c_solver = np.zeros((6, ))
    X_g_solver = np.zeros((6, ))
    
    totalNoLaps = 5
    
    X_final = 0
    prev_s = 0
    
    start = time.time()
    
    if(noOfLapsCompleted==0 and PIDflag==True):
                    
        STATES = np.load('PID_states_11.npz')
        X_c = STATES['X_c'].squeeze()
        X_g = STATES['X_g'].squeeze()
        U = STATES['U'].squeeze()
        
        lmpcpredictiveModel = PredictiveModel(6,2,4)
        for i in range(0,4): # add trajectories used for model learning
            lmpcpredictiveModel.addTrajectory(X_c, U)
            
        lmpcParameters.timeVarying = True 
        lmpc = LMPC(numSS_Points, numSS_it, QterminalSlack, lmpcParameters, lmpcpredictiveModel)
        for i in range(0,4): # add trajectories for safe set
            lmpc.addTrajectory(X_c, U, X_g)
            
        
        X_curv_points = []
        X_glob_points = []
        U_points = []
        
        X_curv_points_model = []
        X_glob_points_model = []
        U_points_model = []
        
        PIDflag==False
        
    newLap = False
    

    if(currentLapTime>=wait_time):
            
        # if((((obs['poses_x'][0]-conf.sx)**2+(obs['poses_y'][0]-conf.sy)**2)**0.5< 0.05 and (simulatorTotalRuntime)>5) or loadedFile==True):
        while not done:
            
            # # if(((obs['poses_x'][0]-conf.sx)**2+(obs['poses_y'][0]-conf.sy)**2)**0.5<0.05 and simulatorTotalRuntime>50 and newLap == True):

            if(len(X_curv_points)==0 and noOfLapsCompleted==0):
                X_c_solver = np.zeros((6, ))
                X_c_solver[0] = 0.5
                X_g_solver = X_c_solver
                X_curv_points = [X_c_solver]
                X_glob_points = [X_g_solver]
                X_curv_points_model = [X_c_solver]
                X_glob_points_model = [X_g_solver]
                
            elif(len(X_curv_points)!=0):
                X_c_solver = X_curv_points[-1]
            
            lmpc.solve(X_c_solver)
            # if(count%10==0 or len(X_curv_points)==1):
            # lmpc.solve(X_c_solver)
            U_points.append(lmpc.uPred[0,:].copy())
            lmpc.addPoint(X_curv_points[-1], U_points[-1])
            
            obs, step_reward, done, info = env.step(np.array([[lmpc.uPred[0,0],lmpc.uPred[0,1]]]))

            
            # if(count%10==0 or obs['X_c'][0][4]>lmpcpredictiveModel.TrackLength):
            X_curv_points.append(obs['X_c'][0])
            X_glob_points.append(obs['X_g'][0])
            
            if count%10==0 or obs['X_c'][0][4]>lmpcpredictiveModel.TrackLength:
                X_curv_points_model.append(obs['X_c'][0])
                X_glob_points_model.append(obs['X_g'][0])
                U_points_model.append(lmpc.uPred[0,:].copy())

            currentLapTime += 0.01
            
            if(done):
                if(noOfLapsCompleted<totalNoLaps):
                    done = False
                
            count += 1
            
            if(obs['X_c'][0][4]>lmpcpredictiveModel.TrackLength and currentLapTime>5):
                newLap = True
        
            simulatorTotalRuntime += step_reward
            env.render(mode='human_fast')
            
            if(newLap == True):
            
                loadedFile=False
                noOfLapsCompleted += 1
                
                print("Lap Completed")
                print("Lap Time: ",currentLapTime)
                lapTimes.append(currentLapTime)
                    
                print("Lap number post PID: ",noOfLapsCompleted)
                print(X_curv_points[-1][4])
                X_final = [X_curv_points[-1] - np.array([0, 0, 0, 0, lmpcpredictiveModel.TrackLength, 0]), np.array(X_glob_points[-1])]
                print(X_final[0])
                X_curv_points.pop()
                X_glob_points.pop()
                
                X_curv_points_model.pop()
                X_glob_points_model.pop()
                
                X_c = np.array(X_curv_points).squeeze()
                X_g = np.array(X_glob_points).squeeze()
                U = np.array(U_points).squeeze()
                
                X_c_model = np.array(X_curv_points_model).squeeze()
                X_g_model = np.array(X_glob_points_model).squeeze()
                U_model = np.array(U_points_model).squeeze()
                
                
                # lmpcpredictiveModel.addTrajectory(X_c_model, U_model)
                # lmpc.addTrajectory(X_c, U, X_g)

                X_curv_points = []
                X_glob_points = []
                U_points = []
                
                X_curv_points_model = []
                X_glob_points_model = []
                U_points_model = []
    
                X_c_solver = X_final[0]
                X_g_solver = X_final[1]
                
                X_c_solver_model = X_final[0]
                X_g_solver_model = X_final[1]
                
                X_curv_points.append(X_final[0])
                X_glob_points.append(X_final[1])
                
                X_curv_points_model.append(X_final[0])
                X_glob_points_model.append(X_final[1])
    
                # noOfLapsCompleted+=1
                if(noOfLapsCompleted==totalNoLaps):
                    done = True
                    break
                
                PIDflag = False
                currentLapTime = 0
                newLap = False
                count = 0
        
    print('Sim elapsed time:', simulatorTotalRuntime, 'Real elapsed time:', time.time()-start)
    print("Lap Times for each lap: ", lapTimes)

    # pr.disable()       
    # s = io.StringIO()
    # sortby = SortKey.CUMULATIVE
    # ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
    # ps.print_stats(0.05)
    # print(s.getvalue())