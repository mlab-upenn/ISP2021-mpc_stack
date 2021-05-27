import time
import yaml
import gym
import osqp
import numpy as np
import scipy as sp
from scipy import sparse
import math
from argparse import Namespace

from scipy.linalg import expm
from scipy.signal import cont2discrete
from scipy import spatial
from scipy.linalg import block_diag

from numba import njit , jit

import matplotlib.pyplot as plt

##### param #####
Df = 0.17
Cf = 1
Bf = 2.58
Dr = 0.19
Br = 3.38
Cm1 = 500
Cm2 = 0.0545
Cr = 1
Cd = 0.001

m = 3.74
Iz = 0.04712
g = 9.81 
h = 0.074
lf = 0.15875 
lr = 0.17145
u_fr = 1.0489
################

@njit(fastmath=False, cache=True)
def nonlinear_dynamics(x1,x2,x3,x4,x5,x6,u1,u2):
  x_dot = np.array([x4*math.cos(x3)-x5*math.sin(x3),
                    x5*math.cos(x3)+x4*math.sin(x3),
                    x6,
                    -(Cd*x4**2 - u2/3.5*(Cm1 - Cm2*x4) + Df*math.sin(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4))))*math.sin(u1) - m*x5*x6 + (g*lf*m*u_fr)/(lf + lr))/(m*((h*u_fr)/(lf + lr) + 1)),
                    -(Dr*math.sin(Cr*math.atan(Br*math.atan((x5 - lr*x6)/x4))) - Df*math.sin(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4))))*math.cos(u1) + m*x4*x6)/m,
                    (Dr*lr*math.sin(Cr*math.atan(Br*math.atan((x5 - lr*x6)/x4))) + Df*lf*math.sin(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4))))*math.cos(u1))/Iz])
  return x_dot


@njit(fastmath=False, cache=True)
def nonlinear_discretize(x1,x2,x3,x4,x5,x6,u1,u2,Ts):
  x_k = np.array([x1,x2,x3,x4,x5,x6])
  x_kplus1 = x_k + Ts*nonlinear_dynamics(x1,x2,x3,x4,x5,x6,u1,u2)
  return x_kplus1

@njit(fastmath=False, cache=True)
def linearize(x1,x2,x3,x4,x5,x6,u1,u2):
  Ac = np.array([[ 0., 0., - x5*math.cos(x3) - x4*math.sin(x3), math.cos(x3),  -math.sin(x3), 0],
                [ 0., 0.,   x4*math.cos(x3) - x5*math.sin(x3),  math.sin(x3), math.cos(x3),  0],
                [ 0., 0., 0., 0., 0., 1.],
                [ 0., 0., 0., -(Cm2*u2/3.5 + 2*Cd*x4 + (Bf*Cf*Df*math.sin(u1)*math.cos(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4))))*(x5 + lf*x6))/(x4**2*((x5 + lf*x6)**2/x4**2 + 1)*(Bf**2*(u1 - math.atan((x5 + lf*x6)/x4))**2 + 1)))/(m*((h*u_fr)/(lf + lr) + 1)), (m*x6 + (Bf*Cf*Df*math.sin(u1)*math.cos(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4)))))/(x4*((x5 + lf*x6)**2/x4**2 + 1)*(Bf**2*(u1 - math.atan((x5 + lf*x6)/x4))**2 + 1)))/(m*((h*u_fr)/(lf + lr) + 1)), (m*x5 + (Bf*Cf*Df*lf*math.sin(u1)*math.cos(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4)))))/(x4*((x5 + lf*x6)**2/x4**2 + 1)*(Bf**2*(u1 - math.atan((x5 + lf*x6)/x4))**2 + 1)))/(m*((h*u_fr)/(lf + lr) + 1))],
                [ 0., 0., 0.,  ((Br*Cr*Dr*math.cos(Cr*math.atan(Br*math.atan((x5 - lr*x6)/x4)))*(x5 - lr*x6))/(x4**2*((x5 - lr*x6)**2/x4**2 + 1)*(Br**2*math.atan((x5 - lr*x6)/x4)**2 + 1)) - m*x6 + (Bf*Cf*Df*math.cos(u1)*math.cos(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4))))*(x5 + lf*x6))/(x4**2*((x5 + lf*x6)**2/x4**2 + 1)*(Bf**2*(u1 - math.atan((x5 + lf*x6)/x4))**2 + 1)))/m,  -((Br*Cr*Dr*math.cos(Cr*math.atan(Br*math.atan((x5 - lr*x6)/x4))))/(x4*((x5 - lr*x6)**2/x4**2 + 1)*(Br**2*math.atan((x5 - lr*x6)/x4)**2 + 1)) + (Bf*Cf*Df*math.cos(u1)*math.cos(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4)))))/(x4*((x5 + lf*x6)**2/x4**2 + 1)*(Bf**2*(u1 - math.atan((x5 + lf*x6)/x4))**2 + 1)))/m, -(m*x4 - (Br*Cr*Dr*lr*math.cos(Cr*math.atan(Br*math.atan((x5 - lr*x6)/x4))))/(x4*((x5 - lr*x6)**2/x4**2 + 1)*(Br**2*math.atan((x5 - lr*x6)/x4)**2 + 1)) + (Bf*Cf*Df*lf*math.cos(u1)*math.cos(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4)))))/(x4*((x5 + lf*x6)**2/x4**2 + 1)*(Bf**2*(u1 - math.atan((x5 + lf*x6)/x4))**2 + 1)))/m],
                [ 0., 0., 0., -((Br*Cr*Dr*lr*math.cos(Cr*math.atan(Br*math.atan((x5 - lr*x6)/x4)))*(x5 - lr*x6))/(x4**2*((x5 - lr*x6)**2/x4**2 + 1)*(Br**2*math.atan((x5 - lr*x6)/x4)**2 + 1)) - (Bf*Cf*Df*lf*math.cos(u1)*math.cos(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4))))*(x5 + lf*x6))/(x4**2*((x5 + lf*x6)**2/x4**2 + 1)*(Bf**2*(u1 - math.atan((x5 + lf*x6)/x4))**2 + 1)))/Iz, ((Br*Cr*Dr*lr*math.cos(Cr*math.atan(Br*math.atan((x5 - lr*x6)/x4))))/(x4*((x5 - lr*x6)**2/x4**2 + 1)*(Br**2*math.atan((x5 - lr*x6)/x4)**2 + 1)) - (Bf*Cf*Df*lf*math.cos(u1)*math.cos(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4)))))/(x4*((x5 + lf*x6)**2/x4**2 + 1)*(Bf**2*(u1 - math.atan((x5 + lf*x6)/x4))**2 + 1)))/Iz,   -((Br*Cr*Dr*lr**2*math.cos(Cr*math.atan(Br*math.atan((x5 - lr*x6)/x4))))/(x4*((x5 - lr*x6)**2/x4**2 + 1)*(Br**2*math.atan((x5 - lr*x6)/x4)**2 + 1)) + (Bf*Cf*Df*lf**2*math.cos(u1)*math.cos(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4)))))/(x4*((x5 + lf*x6)**2/x4**2 + 1)*(Bf**2*(u1 - math.atan((x5 + lf*x6)/x4))**2 + 1)))/Iz]])
  
  Bc = np.array([[ 0., 0.],
                [ 0., 0.],
                [ 0., 0.],
                [ -(Df*math.sin(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4))))*math.cos(u1) + (Bf*Cf*Df*math.sin(u1)*math.cos(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4)))))/(Bf**2*(u1 - math.atan((x5 + lf*x6)/x4))**2 + 1))/(m*((h*u_fr)/(lf + lr) + 1)), (Cm1 - Cm2*x4)/(m*((h*u_fr)/(lf + lr) + 1))],
                [ -(Df*math.sin(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4))))*math.sin(u1) - (Bf*Cf*Df*math.cos(u1)*math.cos(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4)))))/(Bf**2*(u1 - math.atan((x5 + lf*x6)/x4))**2 + 1))/m, 0],
                [ -(Df*lf*math.sin(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4))))*math.sin(u1) - (Bf*Cf*Df*lf*math.cos(u1)*math.cos(Cf*math.atan(Bf*(u1 - math.atan((x5 + lf*x6)/x4)))))/(Bf**2*(u1 - math.atan((x5 + lf*x6)/x4))**2 + 1))/Iz, 0]])
  return Ac,Bc



@njit(fastmath=False, cache=True)
def discretize(Ac,Bc,Ts):
  Ad = np.eye(Ac.shape[0]) + Ts*Ac
  Bd = Ts*Bc
  return Ad,Bd


def QP_solver8(Ad_seq,Bd,Q,s,QN,R,x0,xr_seq,ur,umin,umax,u0,delta_u,M,N_plus_W,N_minus_W,N,Ts):
  [nx, nu] = Bd.shape 

  # Quadratic objective
  P = sparse.block_diag([sparse.kron(sparse.eye(N), Q), QN,
                       sparse.kron(sparse.eye(N), R),sparse.kron(sparse.eye(N), s)], format='csc')
  # Linear objective
  xr_stage = xr_seq[0:N]
  xr_N = xr_seq[N]
  q = np.hstack([(((-Q.dot(xr_stage.T))).T).reshape(-1), -QN.dot(xr_N),
               np.kron(np.ones(N), -R.dot(ur)),np.zeros(N)])
  
  
  
  # Linear dynamics

  diag_As = block_diag(*Ad_seq)
  G = np.zeros([nx*(N+1),nx*(N+1)])
  G[nx:diag_As.shape[0]+nx,:diag_As.shape[1]] = diag_As
  K = np.kron(np.eye(N+1),-np.eye(nx))
  Ax = G + K
  Ax = sparse.csc_matrix(Ax)

  Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), Bd)

  S = sparse.csc_matrix((nx*(N+1), N))
  Aeq = sparse.hstack([Ax, Bu , S])

  ## Linearized system dynamics constraints STACKING ##
  stacking_list = []
  for k in range(0,N):
    stacking_list.append(Ad_seq[k]@xr_seq[k] + Bd@ur - nonlinear_discretize(*xr_seq[k],*ur,Ts))
  linearize_constraint = np.array(stacking_list).reshape(-1)
  
  #####################################################

  leq = np.hstack([-x0, linearize_constraint])
  ueq = leq

  # Input and state constraints
  Aineq = sparse.hstack([sparse.csc_matrix((N*nu, (N+1)*nx)),sparse.eye(N*nu),sparse.csc_matrix((N*nu, N))])
  lineq = np.kron(np.ones(N), umin)
  uineq = np.kron(np.ones(N), umax)

  # Halfspace Constraints
  half_up = np.hstack([np.zeros((N,nx)),block_diag(*M),np.zeros((N,N*nu)),-np.eye(N)])
  half_down = np.hstack([np.zeros((N,nx)),block_diag(*M),np.zeros((N,N*nu)),np.eye(N)])
  A_half = np.vstack([half_up,half_down])

  l_half = np.hstack([np.kron(np.ones(N),-math.inf),N_minus_W])
  u_half = np.hstack([N_plus_W,np.kron(np.ones(N),math.inf)])

  # Slack Constraints
  A_slack = np.hstack([np.zeros((N,(N+1)*nx)),np.zeros((N,N*nu)),np.eye(N)])

  l_slack = np.zeros(N)
  u_slack = np.kron(np.ones(N),math.inf)

  # Input Rate of Change Contraints
  A_delta = np.hstack([np.zeros((N*nu,(N+1)*nx)),np.kron(np.eye(N),np.eye(nu))+np.kron(np.eye(N,k=-1),-np.eye(nu)),np.zeros((N*nu,N))])

  l_delta = np.hstack([u0,np.kron(np.ones(N-1),-delta_u)])
  u_delta = np.hstack([u0,np.kron(np.ones(N-1),delta_u)])

  # OSQP constraints
  A = sparse.vstack([Aeq, Aineq, A_half, A_slack, A_delta], format='csc')
  l = np.hstack([leq, lineq, l_half, l_slack, l_delta])
  u = np.hstack([ueq, uineq, u_half, u_slack, u_delta])

  # Create an OSQP object
  prob = osqp.OSQP()

  # Setup workspace
  prob.setup(P, q, A, l, u, warm_start=True,verbose = False)

  # Solving QP 
  results = prob.solve()

  return results.x


@njit(fastmath=False, cache=True)
def M_N_W(trajectory,C2,C3,dl,dr):
  X = trajectory[:,0]
  Y = trajectory[:,1]
 
  M = []
  N_plus_W = []
  N_minus_W = []
  for i in range(1,len(X)):
    Mi = np.array([(X[i]-C2) , (Y[i]-C3) , 0. , 0. , 0. , 0.])
    Ni = (X[i]-C2)*X[i] + (Y[i]-C3)*Y[i]
    Wi_plus = dr*math.sqrt((X[i]-C2)**2 + (Y[i]-C3)**2)
    Wi_minus = dl*math.sqrt((X[i]-C2)**2 + (Y[i]-C3)**2)
    M.append(Mi)
    N_plus_W.append(Ni+Wi_plus)
    N_minus_W.append(Ni-Wi_minus)
  
  return M,N_plus_W,N_minus_W


@njit(fastmath=False, cache=True)
def trajectory_generator(vx,delta,X_init,Y_init,phi_init,L,Ts,N):
  C1 = phi_init 
  C2 = X_init - L*math.sin(C1)/math.tan(delta)
  C3 = Y_init + L*math.cos(C1)/math.tan(delta)
  phi_dot = (vx/L)*math.tan(delta)
  trajectory = []
  for k in range(0,N):
    phi_k = (vx/L)*math.tan(delta)*Ts*k + C1
    X_k = L*math.sin(phi_k)/math.tan(delta) + C2
    Y_k = -L*math.cos(phi_k)/math.tan(delta) + C3
    trajectory.append(np.array([X_k,Y_k,phi_k,vx,0,phi_dot,delta,C2,C3]))   # just added N here!!!!
  return trajectory


@njit(fastmath=False, cache=True)
def end_coord_gridding3(X_init,Y_init,phi_init,L):
  l = np.array([3.0,1.1])
  r = np.array([3.0,-1.1])
  m1 = np.array([-1.0,0.0])   #[-1.0,0.0]
  m2 = np.array([3.5,0.0])    #[2.0,0.0]
  m3 = np.array([9.0,0.0])
  
  
  
  R = np.array([[math.cos(phi_init) , -math.sin(phi_init)],[math.sin(phi_init) , math.cos(phi_init)]])

  l_R = R@l
  r_R = R@r
  m1_R = R@m1
  m2_R = R@m2
  m3_R = R@m3

  p = np.array([X_init,Y_init])
  s = [p+l_R , p+m1_R, p+m2_R, p+m3_R, p+r_R ]
  return s
@njit(fastmath=False, cache=True)
def trajectory_planner7(vx,angle,D,X_init,Y_init,phi_init,L,Ts,N):
  
  steering_angle = math.atan(L*angle/D)
  #steering_angle = angle
  #print(steering_angle)
  """
  #steering_angle2 = math.atan(distance/5)
  steering_angle2 = 0.2*distance**3
  steering_angle = 1.0*steering_angle1 + 1.0*steering_angle2
  """

  trajectory_chosen = trajectory_generator(vx,steering_angle,X_init,Y_init,phi_init,L,Ts,N)
  return trajectory_chosen

#@njit(fastmath=False, cache=True)
def trajectory_linearization(trajectory_chosen,u1,u2,Ts):
  trajectory_chosen = trajectory_chosen[:trajectory_chosen.shape[0]-1,:]   # No need to linearize at the last waypoint
  Ad_seq = []
  for x_op in trajectory_chosen:
    Ac,Bc = linearize(*x_op,u1,u2)
    Ad , Bd = discretize(Ac,Bc,Ts)
    Ad_seq.append(Ad)

  return Ad_seq , Bd

@njit(fastmath=False, cache=True)
def angle_difference(v1,v2):
  unit_v1 = v1/np.linalg.norm(v1)
  unit_v2 = v2/np.linalg.norm(v2)
  dot_product = np.dot(unit_v1, unit_v2)

  angle = math.acos(dot_product)
  return angle

@njit(fastmath=False, cache=True)
def find_closest_point(p,array):
  track_error_list = []
  for p_array in array:
    track_error = np.linalg.norm(p-p_array)
    track_error_list.append(track_error)

  cross_track_error = min(track_error_list)
  closest_point = array[track_error_list.index(cross_track_error)]
  
  return closest_point, cross_track_error

@njit(fastmath=False, cache=True)
def vector_rotation(vector,rot_angle):
  R = np.array([[math.cos(rot_angle) , -math.sin(rot_angle)],[math.sin(rot_angle) , math.cos(rot_angle)]])
  rotated_vector = R@vector
  return rotated_vector


@njit(fastmath=False, cache=True)
def cross(a,b):
  v1 = np.zeros(3)
  v2 = np.zeros(3)
  v1[:2] = a
  v2[:2] = b
  return np.cross(v1,v2)[2]

@njit(fastmath=False, cache=True)
def turn_detection4(X_init,Y_init,phi_init,array,dis,num):
  array = array[:,0:2]

  p = np.array([X_init,Y_init])
  m1 = np.array([-1.5,0.0])   #[-1.0,0.0]
  m2 = np.array([.5,0.0])    #[2.0,0.0]
  m1_R = vector_rotation(m1,phi_init)
  m2_R = vector_rotation(m2,phi_init)
  p1 = p + m1_R
  p2 = p + m2_R

  track_error = cross_track_error(X_init,Y_init,phi_init,array)
  #print(track_error)
  angle_offset = 1.*track_error**3   #1
  #angle_offset = 0.5*track_error

  vec_trans = np.array([dis,0.])      # for moving points only

  """
 # vec_go = vector_rotation(vec_trans,phi_init-angle_offset)   # re-orientation
  vec_go = vector_rotation(vec_trans,phi_init)   # re-orientation
  
  p1 = np.array([X_init,Y_init])
  p2 = p1 + vec_go
  """
  p1_array,error1 = find_closest_point(p1,array)


  ### Loop Start Here ###
  phi = phi_init
  angle_sum = 0. + angle_offset
  for i in range(num):
    p2_array,error2 = find_closest_point(p2,array)



    v1 = p2 - p1   # from the car
    v2 = p2_array - p1_array # from the track

    if np.array_equal(p1_array,p2_array):
      print(i)
      re1 = angle_offset
      break

    angle_diff = angle_difference(v1,v2)
   
    sign = np.sign(cross(v1,v2))
    if sign == 0:
      sign = 1.
    angle_signed = sign*angle_diff   # check HERE 
    error2 = sign*error2

    angle_sum = angle_sum + angle_signed
    
   
    if i == 2:   #2
      re1 = angle_sum
      


    phi = phi+angle_signed  # correction
    vec_go = vector_rotation(vec_trans,phi)   # correction
    p1 = p2
    p2 = p2+vec_go
    p1_array = p2_array
   # p_list.append(p2)
    
  if np.abs(angle_sum) > 0.5:
    vx = 4
    re1 = re1 - angle_offset
  else:
    vx = 8     #8
  return vx,re1    # CHANGE HERE TO re1

@njit(fastmath=False, cache=True)
def cross_track_error(X_init,Y_init,phi_init,array):
  l = np.array([0.,0.1])
  p = np.array([X_init,Y_init])
  r = np.array([0.,-0.1])

  #R = np.array([[math.cos(phi_init) , -math.sin(phi_init)],[math.sin(phi_init) , math.cos(phi_init)]])
  l_Rot = vector_rotation(l,phi_init)
  r_Rot = vector_rotation(r,phi_init)

  l_trans = p + l_Rot
  r_trans = p + r_Rot

  p_closest, track_error = find_closest_point(p,array)

  dis_l = np.linalg.norm(l_trans - p_closest)
  dis_r = np.linalg.norm(r_trans - p_closest)

  track_error = np.sign(dis_r-dis_l)*track_error
  track_error = track_error     # offset can be placed here
  return track_error


if __name__ == '__main__':

    """
    with open('config_example_map.yaml') as file:
        conf_dict = yaml.load(file, Loader=yaml.FullLoader)
    conf = Namespace(**conf_dict)
    """
    
    waypoint_array = np.genfromtxt('./Melbourne_map_waypoints.csv', delimiter=',')
    #waypoint_array = np.genfromtxt('./IMS_map_waypoints.csv', delimiter=',')


    
    params_dict = {'mu': 2.0489, 
                  'C_Sf':1,
                  'C_Sr':1,
                  'lf': 0.15875,
                  'lr': 0.17145,
                  'h': 0.074, 
                  'm': 3.74, 
                  'I': 0.04712, 
                  's_min': -0.4189, 
                  's_max': 0.4189, 
                  'sv_min': -3.2, 
                  'sv_max': 3.2, 
                  'v_switch':7.319, 
                  'a_max': 9.51, 
                  'v_min':-5.0, 
                  'v_max': 20.0, 
                  'width': 0.31, 
                  'length': 0.58}
  
  

    #env = gym.make('f110_gym:f110-v0',params=params_dict, map=conf.map_path, map_ext=conf.map_ext, num_agents=1)
    env = gym.make('f110_gym:f110-v0',params=params_dict, map='./Melbourne_map', map_ext='.png', num_agents=1)
    #obs, step_reward, done, info = env.reset(np.array([[0, 9, -math.pi/2+0.02],[0, 5, -math.pi/2+0.02]]))      #IMS
    obs, step_reward, done, info = env.reset(np.array([[0, 0, 2*math.pi/3 + 0.28]]))    # Melbourne
    env.render()


    laptime = 0.0
    start = time.time()

    ### Start Here ###
    N = 5
    Ts = 0.03
    nx = 6
    nu = 2
    ns = 1
    dl = 0.01
    dr = 0.01



    Q = np.diag([10, 10, 1, 10, 1, 1.])
    QN = Q
    R = np.diag([200, 200])
    s = 500

    umin = np.array([-0.34906585,-1.5]) 
    umax = np.array([0.34906585,16]) 
    delta_u = np.array([0.6,0.02])


    #xmin = np.array([-500,-500,-2*math.pi,-1.1,-10,-5])
    #xmax = np.array([500,500,2*math.pi,3.5,10,5])


    u0 = np.array([0,0])
   # time_start = time.time()

    while not done:
       # time_now = time.time()
     #   end_coord_list = end_coord_gridding3(obs['poses_x'][0],obs['poses_y'][0],obs['poses_theta'][0],lf+lr)

        vel,angle = turn_detection4(obs['poses_x'][0],obs['poses_y'][0],obs['poses_theta'][0],waypoint_array,1.,7)   #1. , 7
      #  print(vel)
        trajectory_chosen = np.array(trajectory_planner7(vel,angle,1.,obs['poses_x'][0],obs['poses_y'][0],obs['poses_theta'][0],lf+lr,Ts,N)) #1.
        C2 = trajectory_chosen[0,7]
        C3 = trajectory_chosen[0,8]
        

        u2 = trajectory_chosen[0,3]
        u1 = trajectory_chosen[0,6]
        ur = np.array([u1,u2])

       # print(u2)
        
        trajectory_chosen = trajectory_chosen[:,:trajectory_chosen.shape[1]-3]  ## Getting rid of the steering angle, C1 , C2 as the states !!!!
       # print(trajectory_chosen[:,:2])

        M,N_plus_W,N_minus_W = M_N_W(trajectory_chosen,C2,C3,dl,dr)

        
        Ad_seq , Bd = trajectory_linearization(trajectory_chosen,*ur,Ts)

        #x0 = trajectory_chosen[0]
        x0 = np.array([obs['poses_x'][0],obs['poses_y'][0],obs['poses_theta'][0],obs['linear_vels_x'][0]+0.0001,obs['linear_vels_y'][0],obs['ang_vels_z'][0]])

        
        re = QP_solver8(Ad_seq,Bd,Q,s,QN,R,x0,trajectory_chosen,ur,umin,umax,u0,delta_u,M,N_plus_W,N_minus_W,N-1,Ts) #N_here = N-1\
        
        vx_new = re[(N)*nx+nu+1]
        steering_angle_new = re[(N)*nx+nu]
        
        """
        vx_new = re[-(N-1)*(nu+ns)+1]
        steering_angle_new = re[-(N-1)*(nu+ns)]
        print(steering_angle_new)
        """
        if vx_new == None:
          print("lmao")
          vx = u2
          steering_angle = u1
          u0 = np.array([steering_angle,vx])
          
        else:
          vx = vx_new
          steering_angle = steering_angle_new
          vx_old = vx_new
          steering_angle_old = steering_angle_new
          u0 = np.array([steering_angle,vx])
        
        #print(vx)
        """
        if vx_new == None:
          print("lol")
          vx = 0
          steering_angle = 0
          u0 = np.array([0,0])
        else:
          vx = vx_new
          steering_angle = steering_angle_new
          u0 = np.array([steering_angle,vx])
          
        """
        #print(steering_angle)
        #print(vx)
        
        
        """
        if time.time()-time_start<5:
          obs, step_reward, done, info = env.step(np.array([[0,2]]))
        else:
          obs, step_reward, done, info = env.step(np.array([[steering_angle,vx]]))
        """

        obs, step_reward, done, info = env.step(np.array([[steering_angle,vx],[0,0]]))
          


        
        
        laptime += step_reward
        
        env.render(mode='human_fast')
        #time_elapse = time.time() - time_now
        #print(time_elapse)


       # time_elapse = time.time() - time_now
        #time.sleep(0.05-time_elapse)
       # print(time.time() - time_now)
   # print('Sim elapsed time:', laptime, 'Real elapsed time:', time.time()-start)