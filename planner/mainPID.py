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

import sys
sys.path.append('planner/')
waypoints = np.loadtxt("IMS_map_waypoints.csv", delimiter=",")
# sys.path.append('fnc/controller')
# sys.path.append('fnc')

# from planner.initControllerParameters import initMPCParams, initLMPCParams
# from planner.PredictiveControllers import MPC, LMPC, MPCParams
# from planner.PredictiveModel import PredictiveModel

# from initControllerParameters import initMPCParams, initLMPCParams
# from PredictiveControllers import MPC, LMPC, MPCParams
# from PredictiveModel import PredictiveModel



env_name = 'f110_gym:f110-v0'
if env_name in gym.envs.registry.env_specs:

    del gym.envs.registration.registry.env_specs[env_name]
    

if __name__== "__main__":
    # instantiating the environment
    # global waypoints
    with open('config_IMS_map.yaml') as file:
        conf_dict = yaml.load(file, Loader=yaml.FullLoader)
    conf = Namespace(**conf_dict)

    env = gym.make('f110_gym:f110-v0', map=conf.map_path, map_ext=conf.map_ext, num_agents=1)
    obs, step_reward, done, info = env.reset(np.array([[conf.sx, conf.sy, conf.stheta]]))

    
    # simulation loop
    laptime = 0
    noOfLaps = 0 # 1 because i am loading a precomputed file npz
    speed = 0
    steer = 0

    X_curv_points = []
    X_glob_points = []
    U_points = []
    
    PIDflag = True   

    start = time.time()
    total_lap_time = 0
    current_lap_time = 0
    counter = 0
    
    max_time = 500
    
    while not done:

        # if((((obs['poses_x'][0]-conf.sx)**2+(obs['poses_y'][0]-conf.sy)**2)**0.5< 0.05 and (current_lap_time)>5) or loadedFile==True):
        if(current_lap_time>=max_time):
            
            loadedFile=False
            
            print("PID Operation Completed")
                
            X_c = np.array(X_curv_points).squeeze()
            X_g = np.array(X_glob_points).squeeze()
            U = np.array(U_points).squeeze()
            
            # np.savez('PID_states_13.npz', X_g=X_g, X_c=X_c, U=U)

            # noOfLaps+=1
            PIDflag = False
            done = True
            # current_lap_time = 0
            
            
        if(PIDflag==True):
            obs, step_reward, done, info = env.step(np.array([[1000, 1000]]))
            # print("v_x:[%0.4f] | v_y:[%0.4f] | z_t:[%0.4f] | psi:[%0.4f] | epsi:[%0.4f] | ey:[%0.4f]" \
            #       %(np.array(obs['X_c'][0][0]), np.array(obs['X_c'][0][1]), np.array(obs['X_c'][0][2]), np.array(obs['X_g'][0][3]), np.array(obs['X_c'][0][3]),np.array(obs['X_c'][0][5])))
            # print(obs['U'])
            
            if(counter%10==0):
                X_curv_points.append(obs['X_c'])
                X_glob_points.append(obs['X_g'])
                U_points.append(obs['U'])
            
            current_lap_time += 0.01
            total_lap_time += 0.01
            counter = counter+1
            
            # if(((obs['poses_x'][0]-conf.sx)**2+(obs['poses_y'][0]-conf.sy)**2)**0.5< 0.05 and current_lap_time > 5):
            if obs['X_c'][0][4]>waypoints[-1,5] and current_lap_time>5:
                print("Lap completed: ", current_lap_time)
                current_lap_time = 0
            
            if(done):
                if(total_lap_time<max_time):
                    done = False
        
        laptime += step_reward
        env.render(mode='human_fast')
        
        # if(PIDflag==False):
        #     break
        
    print('Sim elapsed time:', laptime, 'Real elapsed time:', time.time()-start)