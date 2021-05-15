# MIT License

# Copyright (c) 2020 Joseph Auckley, Matthew O'Kelly, Aman Sinha, Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.



"""
Prototype of base classes
Replacement of the old RaceCar, Simulator classes in C++
Author: Hongrui Zheng
"""

import numpy as np
from numba import njit
import math

from f110_gym.envs.dynamic_models import vehicle_dynamics_st, nearest_point_on_trajectory, pid
from f110_gym.envs.laser_models import ScanSimulator2D, check_ttc_jit, ray_cast
from f110_gym.envs.collision_models import get_vertices, collision_multiple

class RaceCar(object):
    """
    Base level race car class, handles the physics and laser scan of a single vehicle

    Data Members:
        params (dict): vehicle parameters dictionary
        is_ego (bool): ego identifier
        time_step (float): physics timestep
        num_beams (int): number of beams in laser
        fov (float): field of view of laser
        state (np.ndarray (7, )): state vector [x, y, theta, vel, steer_angle, ang_vel, slip_angle]
        odom (np.ndarray(13, )): odometry vector [x, y, z, qx, qy, qz, qw, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z]
        accel (float): current acceleration input
        steer_angle_vel (float): current steering velocity input
        in_collision (bool): collision indicator

    """

    def __init__(self, params, seed, is_ego=False, time_step=0.01, num_beams=1080, fov=4.7):
        """
        Init function

        Args:
            params (dict): vehicle parameter dictionary, includes {'mu', 'C_Sf', 'C_Sr', 'lf', 'lr', 'h', 'm', 'I', 's_min', 's_max', 'sv_min', 'sv_max', 'v_switch', 'a_max': 9.51, 'v_min', 'v_max', 'length', 'width'}
            is_ego (bool, default=False): ego identifier
            time_step (float, default=0.01): physics sim time step
            num_beams (int, default=1080): number of beams in the laser scan
            fov (float, default=4.7): field of view of the laser

        Returns:
            None
        """

        # initialization
        self.params = params
        self.seed = seed
        self.is_ego = is_ego
        self.time_step = time_step
        self.num_beams = 1080
        self.fov = 4.7

        self.X = np.zeros((6, ))
        self.X[0] = 0.5
        self.X_curv = np.zeros((6, ))
        self.X_curv[0] = 0.5
        
        self.flag = False
        
        # pose of opponents in the world
        self.opp_poses = None

        # control inputs
        self.accel = 0.0
        self.steer_angle_vel = 0.0
        self.U = np.array([[0.0,0.0]])

        # steering delay buffer
        self.steer_buffer = np.empty((0, ))
        self.steer_buffer_size = 2

        # collision identifier
        self.in_collision = False

        # collision threshold for iTTC to environment
        self.ttc_thresh = 0.005

        # initialize scan sim
        self.scan_simulator = ScanSimulator2D(num_beams, fov, seed=self.seed)
        scan_ang_incr = self.scan_simulator.get_increment()

        # current scan
        self.current_scan = np.zeros((num_beams, ))

        # angles of each scan beam, distance from lidar to edge of car at each beam, and precomputed cosines of each angle
        self.cosines = np.zeros((num_beams, ))
        self.scan_angles = np.zeros((num_beams, ))
        self.side_distances = np.zeros((num_beams, ))

        dist_sides = params['width']/2.
        dist_fr = (params['lf']+params['lr'])/2.

        for i in range(num_beams):
            angle = -fov/2. + i*scan_ang_incr
            self.scan_angles[i] = angle
            self.cosines[i] = np.cos(angle)

            if angle > 0:
                if angle < np.pi/2:
                    # between 0 and pi/2
                    to_side = dist_sides / np.sin(angle)
                    to_fr = dist_fr / np.cos(angle)
                    self.side_distances[i] = min(to_side, to_fr)
                else:
                    # between pi/2 and pi
                    to_side = dist_sides / np.cos(angle - np.pi/2.)
                    to_fr = dist_fr / np.sin(angle - np.pi/2.)
                    self.side_distances[i] = min(to_side, to_fr)
            else:
                if angle > -np.pi/2:
                    # between 0 and -pi/2
                    to_side = dist_sides / np.sin(-angle)
                    to_fr = dist_fr / np.cos(-angle)
                    self.side_distances[i] = min(to_side, to_fr)
                else:
                    # between -pi/2 and -pi
                    to_side = dist_sides / np.cos(-angle - np.pi/2)
                    to_fr = dist_fr / np.sin(-angle - np.pi/2)
                    self.side_distances[i] = min(to_side, to_fr)
                    
        self.waypoints = np.loadtxt("IMS_map_waypoints.csv", delimiter=",")

    def update_params(self, params):
        """
        Updates the physical parameters of the vehicle
        Note that does not need to be called at initialization of class anymore

        Args:
            params (dict): new parameters for the vehicle

        Returns:
            None
        """
        self.params = params
    
    def set_map(self, map_path, map_ext):
        """
        Sets the map for scan simulator
        
        Args:
            map_path (str): absolute path to the map yaml file
            map_ext (str): extension of the map image file
        """
        self.scan_simulator.set_map(map_path, map_ext)

    def reset(self, pose):
        """
        Resets the vehicle to a pose
        
        Args:
            pose (np.ndarray (3, )): pose to reset the vehicle to

        Returns:
            None
        """
        # clear control inputs
        self.accel = 0.0
        self.steer_angle_vel = 0.0
        # clear collision indicator
        self.in_collision = False

        # x - NEW STATE
        self.X = np.zeros((6,))
        self.X[0] = 0.5
        self.X[4:6] = pose[0:2]
        self.X[3] = pose[2]

        # x - NEW GLOBAL STATE
        self.X_curv = np.zeros((6,))
        self.X_curv[0] = 0.5

        # reset scan random generator
        self.scan_simulator.reset_rng(self.seed)

    def ray_cast_agents(self, scan):
        """
        Ray cast onto other agents in the env, modify original scan

        Args:
            scan (np.ndarray, (n, )): original scan range array

        Returns:
            new_scan (np.ndarray, (n, )): modified scan
        """

        # starting from original scan
        new_scan = scan
        
        # loop over all opponent vehicle poses
        for opp_pose in self.opp_poses:
            # get vertices of current oppoenent
            opp_vertices = get_vertices(opp_pose, self.params['length'], self.params['width'])
            new_scan = ray_cast(np.append(self.X[4:6], self.X[3]), new_scan, self.scan_angles, opp_vertices)

        return new_scan

    def check_ttc(self):
        """
        Check iTTC against the environment, sets vehicle states accordingly if collision occurs.
        Note that this does NOT check collision with other agents.

        state is [x, y, steer_angle, vel, yaw_angle, yaw_rate, slip_angle]

        Args:
            None

        Returns:
            None
        """
        
        in_collision = check_ttc_jit(self.current_scan, self.X[0], self.scan_angles, self.cosines, self.side_distances, self.ttc_thresh)
        # if in collision stop vehicle
        if in_collision:
            self.X[0] = 0
            self.accel = 0.0
            self.steer_angle_vel = 0.0

        # update state
        self.in_collision = in_collision

        return in_collision

    def update_pose(self, raw_steer, vel):
        """
        Steps the vehicle's physical simulation

        Args:
            steer (float): desired steering angle
            vel (float): desired longitudinal velocity

        Returns:
            None
        """
        
        if(raw_steer==1000):
            self.U,dist = pid(1, self.X_curv,self.X)
        else:
            # run lmpc code
            # call MPC function here-------------------------
            self.U[0,0] = raw_steer # steering rate
            self.U[0,1] = vel # acceleration

        f_curv,f = vehicle_dynamics_st(self.X_curv,self.X,self.U)

        # update state
        self.X = self.X + f * self.time_step
        self.X_curv = self.X_curv + f_curv * self.time_step
        
        position = np.array([self.X[4],self.X[5]])

        wpts = np.vstack((self.waypoints[:, 0], self.waypoints[:, 1])).T
        _,_,_,nearest_point_id,intersected_point = nearest_point_on_trajectory(position, wpts)
        dist = ((self.X[4]-intersected_point[0])**2 + (self.X[5]-intersected_point[1])**2)**0.5
        
        sign = 0
        if(nearest_point_id==0):
            sign = (self.X[4]-self.waypoints[nearest_point_id,0])*(self.waypoints[nearest_point_id+1,1]-self.waypoints[nearest_point_id,1]) - (self.X[5]-self.waypoints[nearest_point_id,1])*(self.waypoints[nearest_point_id+1,0]-self.waypoints[nearest_point_id,0])
        
        else:
            sign = (self.X[4]-self.waypoints[nearest_point_id,0])*(self.waypoints[nearest_point_id-1,1]-self.waypoints[nearest_point_id,1]) - \
                (self.X[5]-self.waypoints[nearest_point_id,1])*(self.waypoints[nearest_point_id-1,0]-self.waypoints[nearest_point_id,0])
        
        e_y = np.sign(sign)*dist
        self.X_curv[5] = e_y
        
        # if(raw_steer==1000):
        #     self.X_curv[5] = dist
        
        
        index = 3            
        # if self.X_curv[index] > np.pi:
        #     self.X_curv[index] = self.X_curv[index]%np.pi - np.pi
        # elif self.X_curv[index] <= -np.pi:
        #     self.X_curv[index] = self.X_curv[index]%np.pi + np.pi
            
        if self.X[index] > np.pi:
            self.X[index] = self.X[index]%np.pi - np.pi
        elif self.X[index] < -np.pi:
            self.X[index] = self.X[index]%np.pi
                  
        if self.X_curv[4]>self.waypoints[-1,5]:
            if(self.flag==False):
                self.flag = True
                pass
            elif(self.flag == True):
                self.X_curv[4] = self.X_curv[4]%self.waypoints[-1,5]
                self.flag = False
        # index = 2      
        # if(self.X_curv[index]>np.pi):
        #     self.X_curv[index] = (self.X_curv[index]%np.pi) - np.pi
        # elif(self.X_curv[index] < -np.pi):
        #     self.X_curv[index] = (self.X_curv[index]%np.pi) + np.pi
        
        # if(self.X[index]>np.pi):
        #     self.X[index] = (self.X[index]%np.pi) - np.pi
        # elif(self.X[index] < -np.pi):
        #     self.X[index] = (self.X[index]%np.pi) + np.pi
        
        # Noises
        # noise_vx = np.max([-0.05, np.min([np.random.randn() * 0.01, 0.05])])
        # noise_vy = np.max([-0.05, np.min([np.random.randn() * 0.01, 0.05])])
        # noise_wz = np.max([-0.05, np.min([np.random.randn() * 0.005, 0.05])])


        # self.X_curv[0] = self.X_curv[0] + 0.01*noise_vx
        # self.X_curv[1] = self.X_curv[1] + 0.01*noise_vy
        # self.X_curv[2] = self.X_curv[2] + 0.01*noise_wz

        # update scan
        self.current_scan = self.scan_simulator.scan(np.append(self.X[4:6], self.X[3]))

    def update_opp_poses(self, opp_poses):
        """
        Updates the vehicle's information on other vehicles

        Args:
            opp_poses (np.ndarray(num_other_agents, 3)): updated poses of other agents

        Returns:
            None
        """
        self.opp_poses = opp_poses


    def update_scan(self):
        """
        Steps the vehicle's laser scan simulation
        Separated from update_pose because needs to update scan based on NEW poses of agents in the environment

        Args:
            None

        Returns:
            None
        """
        
        # check ttc
        self.check_ttc()

        # ray cast other agents to modify scan
        self.current_scan = self.ray_cast_agents(self.current_scan)



class Simulator(object):
    """
    Simulator class, handles the interaction and update of all vehicles in the environment

    Data Members:
        num_agents (int): number of agents in the environment
        time_step (float): physics time step
        agent_poses (np.ndarray(num_agents, 3)): all poses of all agents
        agents (list[RaceCar]): container for RaceCar objects
        collisions (np.ndarray(num_agents, )): array of collision indicator for each agent
        collision_idx (np.ndarray(num_agents, )): which agent is each agent in collision with

    """

    def __init__(self, params, num_agents, seed, time_step=0.01, ego_idx=0):
        """
        Init function

        Args:
            params (dict): vehicle parameter dictionary, includes {'mu', 'C_Sf', 'C_Sr', 'lf', 'lr', 'h', 'm', 'I', 's_min', 's_max', 'sv_min', 'sv_max', 'v_switch', 'a_max', 'v_min', 'v_max', 'length', 'width'}
            num_agents (int): number of agents in the environment
            seed (int): seed of the rng in scan simulation
            time_step (float, default=0.01): physics time step
            ego_idx (int, default=0): ego vehicle's index in list of agents

        Returns:
            None
        """
        self.num_agents = num_agents
        self.seed = seed
        self.time_step = time_step
        self.ego_idx = ego_idx
        self.params = params
        self.agent_poses = np.empty((self.num_agents, 3))
        self.agents = []
        self.collisions = np.zeros((self.num_agents, ))
        self.collision_idx = -1 * np.ones((self.num_agents, ))

        # initializing agents
        for i in range(self.num_agents):
            if i == ego_idx:
                ego_car = RaceCar(params, self.seed, is_ego=True)
                self.agents.append(ego_car)
            else:
                agent = RaceCar(params, self.seed)
                self.agents.append(agent)

    def set_map(self, map_path, map_ext):
        """
        Sets the map of the environment and sets the map for scan simulator of each agent

        Args:
            map_path (str): path to the map yaml file
            map_ext (str): extension for the map image file

        Returns:
            None
        """
        for agent in self.agents:
            agent.set_map(map_path, map_ext)


    def update_params(self, params, agent_idx=-1):
        """
        Updates the params of agents, if an index of an agent is given, update only that agent's params

        Args:
            params (dict): dictionary of params, see details in docstring of __init__
            agent_idx (int, default=-1): index for agent that needs param update, if negative, update all agents

        Returns:
            None
        """
        if agent_idx < 0:
            # update params for all
            for agent in self.agents:
                agent.update_params(params)
        elif agent_idx >= 0 and agent_idx < self.num_agents:
            # only update one agent's params
            self.agents[agent_idx].update_params(params)
        else:
            # index out of bounds, throw error
            raise IndexError('Index given is out of bounds for list of agents.')

    def check_collision(self):
        """
        Checks for collision between agents using GJK and agents' body vertices

        Args:
            None

        Returns:
            None
        """
        # get vertices of all agents
        all_vertices = np.empty((self.num_agents, 4, 2))
        for i in range(self.num_agents):
            # all_vertices[i, :, :] = get_vertices(np.append(self.agents[i].state[0:2],self.agents[i].state[4]), self.params['length'], self.params['width'])
            all_vertices[i, :, :] = get_vertices(np.append(self.agents[i].X[4:6],self.agents[i].X[3]), self.params['length'], self.params['width'])
        self.collisions, self.collision_idx = collision_multiple(all_vertices)


    def step(self, control_inputs):
        """
        Steps the simulation environment

        Args:
            control_inputs (np.ndarray (num_agents, 2)): control inputs of all agents, first column is desired steering angle, second column is desired velocity
        
        Returns:
            observations (dict): dictionary for observations: poses of agents, current laser scan of each agent, collision indicators, etc.
        """
        
        # looping over agents
        for i, agent in enumerate(self.agents):
            # update each agent's pose
            agent.update_pose(control_inputs[i, 0], control_inputs[i, 1])

            # update sim's information of agent poses
            self.agent_poses[i, :] = np.append(agent.X[4:6], agent.X[3])

        # check collisions between all agents
        self.check_collision()


        for i, agent in enumerate(self.agents):
            # update agent's information on other agents
            opp_poses = np.concatenate((self.agent_poses[0:i, :], self.agent_poses[i+1:, :]), axis=0)
            agent.update_opp_poses(opp_poses)

            # update each agent's current scan based on other agents
            agent.update_scan()

            # update agent collision with environment
            if agent.in_collision:
                self.collisions[i] = 1.

        # fill in observations
        # state is [x, y, steer_angle, vel, yaw_angle, yaw_rate, slip_angle]
        # new state comprises of two representations: one in inertial frame and the other one
        # in curvillinear frame
        # collision_angles is removed from observations
        observations = {'ego_idx': self.ego_idx,
            'scans': [],
            'inertial': [],
            'curv': [],
            'poses_x': [],
            'poses_y': [],
            'poses_theta': [],
            'linear_vels_x': [],
            'linear_vels_y': [],
            'ang_vels_z': [],
            'collisions': self.collisions,
            'X_c': [],
            'X_g': [],
            'U': []}
        for agent in self.agents:
            observations['scans'].append(agent.current_scan)
            observations['inertial'].append(agent.X)
            observations['curv'].append(agent.X_curv)
            # -- retrofitted values based on new dynamic model
            observations['poses_x'].append(agent.X[4])
            observations['poses_y'].append(agent.X[5])
            observations['poses_theta'].append(agent.X[3])
            observations['linear_vels_x'].append(agent.X[0])
            observations['linear_vels_y'].append(agent.X[1])
            observations['ang_vels_z'].append(agent.X[2])
            observations['X_c'].append(agent.X_curv)
            observations['X_g'].append(agent.X)
            observations['U'].append(agent.U)

        return observations

    def reset(self, poses):
        """
        Resets the simulation environment by given poses

        Arges:
            poses (np.ndarray (num_agents, 3)): poses to reset agents to

        Returns:
            None
        """
        
        if poses.shape[0] != self.num_agents:
            raise ValueError('Number of poses for reset does not match number of agents.')

        # loop over poses to reset
        for i in range(self.num_agents):
            self.agents[i].reset(poses[i, :])
