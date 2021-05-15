from cvxopt import spmatrix, matrix, solvers
from numpy import linalg as la
from cvxopt.solvers import qp
import numpy as np
import datetime
import pdb


# This class is not generic and is tailored to the autonomous racing problem.
# The only method need the LT-MPC and the LMPC is regressionAndLinearization, which given a state-action pair
# compute the matrices A,B,C such that x_{k+1} = A x_k + Bu_k + C

class PredictiveModel():
    def __init__(self,  n, d, trToUse):
        # self.map = map
        self.n = n # state dimension
        self.d = d # input dimention
        self.xStored = []
        self.uStored = []
        self.MaxNumPoint = 7 # max number of point per lap to use 
        self.h = 20 # bandwidth of the Kernel for local linear regression
        self.norm_limit = 1000
        self.lamb = 0.01 # regularization
        self.dt = 0.1

        
        self.scaling = np.array([[1.0, 0.0, 0.0, 0.0, 0.0],
                                    [0.0, 1.0, 0.0, 0.0, 0.0],
                                    [0.0, 0.0, 1.0, 0.0, 0.0],
                                    [0.0, 0.0, 0.0, 1.0, 0.0],
                                    [0.0, 0.0, 0.0, 0.0, 1.0]])
        
        # self.scaling = np.array([[1.0]])

        self.stateFeatures    = [0, 1, 2]
        self.inputFeaturesVx  = [1] # these are the indices corresponding to the positions on state matrix
        self.inputFeaturesLat = [0]
        self.usedIt = [i for i in range(trToUse)] # integer
        self.lapTime = []
        
        self.waypoints = np.loadtxt("IMS_map_waypoints.csv", delimiter=",")
        self.TrackLength = self.waypoints[-1,5]

    
    def nearest_point_on_trajectory(self,point, trajectory):
        '''
        Return the nearest point along the given piecewise linear trajectory.
    
        Same as nearest_point_on_line_segment, but vectorized. This method is quite fast, time constraints should
        not be an issue so long as trajectories are not insanely long.
    
            Order of magnitude: trajectory length: 1000 --> 0.0002 second computation (5000fps)
    
        point: size 2 numpy array
        trajectory: Nx2 matrix of (x,y) trajectory waypoints
            - these must be unique. If they are not unique, a divide by 0 error will destroy the world
        '''
        diffs = trajectory[1:,:] - trajectory[:-1,:]
        l2s   = diffs[:,0]**2 + diffs[:,1]**2
        # this is equivalent to the elementwise dot product

        dots = np.empty((trajectory.shape[0]-1, ))
        for i in range(dots.shape[0]):
            dots[i] = np.dot((point - trajectory[i, :]), diffs[i, :])
        t = dots / l2s
        t[t<0.0] = 0.0
        t[t>1.0] = 1.0

        projections = trajectory[:-1,:] + (t*diffs.T).T

        dists = np.empty((projections.shape[0],))
        for i in range(dists.shape[0]):
            temp = point - projections[i]
            dists[i] = np.sqrt(np.sum(temp*temp))
        min_dist_segment = np.argmin(dists)
        
        dists_args_sorted = np.argsort(dists)
        slope = (trajectory[dists_args_sorted[0],1]-trajectory[dists_args_sorted[1],1])/(trajectory[dists_args_sorted[0],0]-trajectory[dists_args_sorted[1],0])
        perpendicular_slope = -1/slope
        
        intersected_point = np.zeros(2)
        intersected_point[0] = (point[1] - trajectory[min_dist_segment,1] + slope * trajectory[min_dist_segment,0] -\
                                    perpendicular_slope*point[0])/(slope-perpendicular_slope)
        intersected_point[1] = point[1] + perpendicular_slope*(intersected_point[0]-point[0])
        
        return projections[min_dist_segment], dists[min_dist_segment], t[min_dist_segment], min_dist_segment,intersected_point


    def getCurvature(self,X,Y):
        position = np.array([X,Y])
        wpts = np.vstack((self.waypoints[:, 0], self.waypoints[:, 1])).T
        _,_,_,nearest_point_id,_ = self.nearest_point_on_trajectory(position, wpts)
        cur = self.waypoints[nearest_point_id,4]
        return cur,nearest_point_id,wpts
    
    def getCurvature_S(self,s):
        diff_args = np.argsort(np.abs(self.waypoints[:,5]-s))
        curv = ((s-self.waypoints[diff_args[0],5])/(self.waypoints[diff_args[1],5] - self.waypoints[diff_args[0],5]))\
                *(self.waypoints[diff_args[1],4] - self.waypoints[diff_args[0],4]) + self.waypoints[diff_args[0],4]
        return curv

    def addTrajectory(self, x, u):
        if self.lapTime == [] or x.shape[0] >= self.lapTime[-1]:
            self.xStored.append(x)
            self.uStored.append(u)
            self.lapTime.append(x.shape[0])
        else:
            for i in range(0, len(self.xStored)):
                if x.shape[0] < self.lapTime[i]:
                    self.xStored.insert(i, x)
                    self.uStored.insert(i, u) 
                    self.lapTime.insert(i, x.shape[0])
                    break

    def regressionAndLinearization(self, x, u):
        Ai = np.zeros((self.n, self.n))
        Bi = np.zeros((self.n, self.d))
        Ci = np.zeros(self.n)

        # Compute Index to use for each stored lap
        xuLin = np.hstack((x[self.stateFeatures], u[:]))
        xLin = x[4]
        self.indexSelected = []
        self.K = []
        for ii in self.usedIt:
            # indexSelected_i, K_i = self.computeIndices(xLin, xuLin, ii)
            indexSelected_i, K_i = self.computeIndices(xuLin, ii)
            self.indexSelected.append(indexSelected_i)
            self.K.append(K_i)
            # print(indexSelected_i)

        # =========================
        # ====== Identify vx ======
        Q_vx, M_vx = self.compute_Q_M(self.inputFeaturesVx, self.usedIt)

        yIndex = 0
        b_vx = self.compute_b(yIndex, self.usedIt, M_vx)
        Ai[yIndex, self.stateFeatures], Bi[yIndex, self.inputFeaturesVx], Ci[yIndex] = self.LMPC_LocLinReg(Q_vx, b_vx, self.inputFeaturesVx)

        # =======================================
        # ====== Identify Lateral Dynamics ======
        Q_lat, M_lat = self.compute_Q_M(self.inputFeaturesLat, self.usedIt)

        yIndex = 1  # vy
        b_vy = self.compute_b(yIndex, self.usedIt, M_lat)
        Ai[yIndex, self.stateFeatures], Bi[yIndex, self.inputFeaturesLat], Ci[yIndex] = self.LMPC_LocLinReg(Q_lat, b_vy, self.inputFeaturesLat)

        yIndex = 2  # wz
        b_wz = self.compute_b(yIndex, self.usedIt, M_lat)
        Ai[yIndex, self.stateFeatures], Bi[yIndex, self.inputFeaturesLat], Ci[yIndex] = self.LMPC_LocLinReg(Q_lat, b_wz, self.inputFeaturesLat)

        # ===========================
        # ===== Linearization =======
        vx = x[0]; vy   = x[1]
        wz = x[2]; epsi = x[3]
        s  = x[4]; ey   = x[5]
        dt = self.dt

        if s < 0:
            print("s is negative, here the state: \n", x)

        startTimer = datetime.datetime.now()  # Start timer for LMPC iteration

        cur = self.getCurvature_S(s)
        # print("Curvature:%0.4f"%cur)

        den = 1 - cur * ey

        # ===========================
        # ===== Linearize epsi ======
        # epsi_{k+1} = epsi + dt * ( wz - (vx * np.cos(epsi) - vy * np.sin(epsi)) / (1 - cur * ey) * cur )
        depsi_vx   = -dt * np.cos(epsi) / den * cur
        depsi_vy   = dt * np.sin(epsi) / den * cur
        depsi_wz   = dt
        depsi_epsi = 1 - dt * (-vx * np.sin(epsi) - vy * np.cos(epsi)) / den * cur
        depsi_s    = 0  # Because cur = constant
        depsi_ey   = dt * (vx * np.cos(epsi) - vy * np.sin(epsi)) / (den ** 2) * cur * (-cur)

        Ai[3, :] = [depsi_vx, depsi_vy, depsi_wz, depsi_epsi, depsi_s, depsi_ey]
        Ci[3]    = epsi + dt * (wz - (vx * np.cos(epsi) - vy * np.sin(epsi)) / (1 - cur * ey) * cur) - np.dot(Ai[3, :], x)
        
        # ===========================
        # ===== Linearize s =========
        # s_{k+1} = s    + dt * ( (vx * np.cos(epsi) - vy * np.sin(epsi)) / (1 - cur * ey) )
        ds_vx   = dt * (np.cos(epsi) / den)
        ds_vy   = -dt * (np.sin(epsi) / den)
        ds_wz   = 0
        ds_epsi = dt * (-vx * np.sin(epsi) - vy * np.cos(epsi)) / den
        ds_s    = 1  # + Ts * (Vx * cos(epsi) - Vy * sin(epsi)) / (1 - ey * rho) ^ 2 * (-ey * drho);
        ds_ey   = -dt * (vx * np.cos(epsi) - vy * np.sin(epsi)) / (den ** 2) * (-cur)

        Ai[4, :] = [ds_vx, ds_vy, ds_wz, ds_epsi, ds_s, ds_ey]
        Ci[4]    = s + dt * ((vx * np.cos(epsi) - vy * np.sin(epsi)) / (1 - cur * ey)) - np.dot(Ai[4, :], x)

        # ===========================
        # ===== Linearize ey ========
        # ey_{k+1} = ey + dt * (vx * np.sin(epsi) + vy * np.cos(epsi))
        dey_vx   = dt * np.sin(epsi)
        dey_vy   = dt * np.cos(epsi)
        dey_wz   = 0
        dey_epsi = dt * (vx * np.cos(epsi) - vy * np.sin(epsi))
        dey_s    = 0
        dey_ey   = 1

        Ai[5, :] = [dey_vx, dey_vy, dey_wz, dey_epsi, dey_s, dey_ey]
        Ci[5]    = ey + dt * (vx * np.sin(epsi) + vy * np.cos(epsi)) - np.dot(Ai[5, :], x)

        endTimer = datetime.datetime.now(); deltaTimer_tv = endTimer - startTimer

        return Ai, Bi, Ci

    def compute_Q_M(self, inputFeatures, usedIt):
        Counter = 0
        X0   = np.empty((0,len(self.stateFeatures)+len(inputFeatures)))
        Ktot = np.empty((0))
        
        for it in usedIt:
            
            X0 = np.append( X0, np.hstack((self.xStored[it][np.ix_(self.indexSelected[Counter], self.stateFeatures)],self.uStored[it][np.ix_(self.indexSelected[Counter], inputFeatures)])), axis=0 )
            Ktot    = np.append(Ktot, self.K[Counter])
            Counter += 1
        # print(self.xStored[0][np.ix_(self.indexSelected[Counter-1],[4])])
        if(len(X0)==0):
            print("Len of X0: %d" %(len(X0)))
        M = np.hstack( (X0, np.ones((X0.shape[0], 1))) )
        # print(M)
        Q0 = np.dot(np.dot(M.T, np.diag(Ktot)), M)
        # print("Rank: %d"%np.linalg.matrix_rank(Q0))
        Q = matrix(Q0 + self.lamb * np.eye(Q0.shape[0]))
        # print(self.indexSelected[Counter-1])

        return Q, M

    def compute_b(self, yIndex, usedIt, M):
        Counter = 0
        y = np.empty((0))
        Ktot = np.empty((0))

        for it in usedIt:
            y       = np.append(y, np.squeeze(self.xStored[it][self.indexSelected[Counter] + 1, yIndex]))
            Ktot    = np.append(Ktot, self.K[Counter])
            Counter += 1

        b = matrix(-np.dot(np.dot(M.T, np.diag(Ktot)), y))
        return b

    def LMPC_LocLinReg(self, Q, b, inputFeatures):
        # Solve QP
        if(np.linalg.matrix_rank(Q)<5):
            print(np.linalg.matrix_rank(Q))
        res_cons = qp(Q, b) # This is ordered as [A B C]
        # Unpack results
        result = np.squeeze(np.array(res_cons['x']))
        A = result[0:len(self.stateFeatures)]
        B = result[len(self.stateFeatures):(len(self.stateFeatures)+len(inputFeatures))]
        C = result[-1]
        return A, B, C

    def computeIndices(self, x, it):
        oneVec = np.ones( (self.xStored[it].shape[0]-1, 1) )
        xVec = (np.dot( np.array([x]).T, oneVec.T )).T
        DataMatrix = np.hstack((self.xStored[it][0:-1, self.stateFeatures], self.uStored[it][0:-1, :]))
        # DataMatrix = self.xStored[it][0:-1, 4]
        
        diff  = np.dot(( DataMatrix - xVec ), self.scaling)
        norm = la.norm(diff, 1, axis=1)
        indexTot =  np.squeeze(np.where(norm < self.norm_limit))

        if (indexTot.shape[0] >= self.MaxNumPoint):
            index = np.argsort(norm)[0:self.MaxNumPoint]
        else:
            index = indexTot
        
        K  = ( 1 - ( norm[index] / self.h )**2 ) * 3/4

        return index, K
    
    # def computeIndices(self, x, xu, it):
    #     oneVec = np.ones( (self.xStored[it].shape[0]-1, 1) )
    #     xVec = (np.dot( np.array([x]).T, oneVec.T )).reshape(-1,1)
    #     # DataMatrix = np.hstack((self.xStored[it][0:-1, self.stateFeatures], self.uStored[it][0:-1, :]))
    #     DataMatrix = self.xStored[it][0:-1, 4].reshape(-1,1)

    #     diff  = np.dot(( DataMatrix - xVec ), self.scaling)
    #     norm = la.norm(diff, 1, axis=1)
    #     indexTot = np.squeeze(np.where(norm < self.h))
    #     # indexTot1 = np.squeeze(np.where(diff>0))
    #     # indexTot2 =  np.squeeze(np.where(norm < self.h))
    #     # indexTot = np.intersect1d(indexTot1, indexTot2)

    #     if (indexTot.shape[0] >= self.MaxNumPoint):
    #         # index = indexTot[1:self.MaxNumPoint+1]
    #         index = np.argsort(norm)[0:self.MaxNumPoint]
    #         # indices = [0,6,13,20,27,34]
    #         # index = np.argsort(norm)
    #         # index = index[([i for i in indices])]
            
    #     else:
    #         index = indexTot
        
    #     oneVec_2 = np.ones( (self.xStored[it].shape[0]-1, 1) )
    #     xVec_2 = ((np.dot( np.array([xu]).T, oneVec_2.T )).T)
    #     DataMatrix_2 = np.hstack((self.xStored[it][0:-1, self.stateFeatures], self.uStored[it][0:-1, :]))
    #     diff_2  = np.dot(( DataMatrix_2 - xVec_2 ), self.scaling_2)
    #     norm_2 = la.norm(diff_2, 1, axis=1)

    #     K  = ( 1 - ( norm_2[index] / self.h )**2 ) * 3/4
    #     # if norm.shape[0]<500:
    #     #     print("norm: ", norm, norm.shape)

    #     return index, K