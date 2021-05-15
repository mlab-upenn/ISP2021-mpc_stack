import numpy as np
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt
import yaml
from argparse import Namespace
import pdb
import sys
np.set_printoptions(threshold = sys.maxsize)
from trajectory_planning_helpers.calc_head_curv_num import calc_head_curv_num
import time

# define pts from the question
with open('config_IMS_map.yaml') as file:
        conf_dict = yaml.load(file, Loader=yaml.FullLoader)
conf = Namespace(**conf_dict)


a = np.loadtxt("IMS_map_waypoints_OLD_1.csv", delimiter=',',skiprows=1)
# b = np.loadtxt("IMS_raceline.csv", delimiter=';')

#---------------
ele_dist = np.zeros(a.shape[0])
ele_dist[1:] = ((a[1:, conf.wpt_xind] - a[0:-1, conf.wpt_xind])**2 + (a[1:, conf.wpt_yind] - a[0:-1, conf.wpt_yind])**2)**0.5

psi, kappa = calc_head_curv_num(a, ele_dist, True, 1, 1, 2, 2, True)
# kappa  = np.round(kappa,4)
# print(kappa)
# a = np.round(a,4)
# print(a[0:10,:])
dist = np.zeros((a.shape[0],1))
const = 0
for i in range(a.shape[0]-1):
    dist[i+1,0] = dist[i,0] + ((a[i+1,0]-a[i,0])**2 + (a[i+1,1]-a[i,1])**2)**0.5


a_new = np.hstack((a,kappa.reshape((-1,1))))
a_new = np.hstack((a_new,dist))

diff_args = np.argsort(np.abs(a_new[:,5]-105))


# pdb.set_trace()
# curv = ((105-a_new[diff_args[0],5])/(a_new[diff_args[1],5] - a_new[diff_args[0],5]))\
#                 *(a_new[diff_args[1],4] - a_new[diff_args[0],4]) + a_new[diff_args[0],4]
# print(curv)
np.savetxt("IMS_map_waypoints.csv", a_new, delimiter=",")

plt.plot(a_new[:,0],a_new[:,1])
plt.plot(b[:,1],b[:,2])
plt.show()