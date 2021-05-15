# Learning Model Predictive Controller (LMPC) on The F1TENTH Gym environment

This is the repository for the LMPC implementation on the F1TENTH Gym environment.


The documentation of the simulation environment can be found [here](https://f1tenth-gym.readthedocs.io/en/latest/).

## Quickstart
**You can install the environment by running**

```bash
$ git clone https://github.com/f1tenth/f1tenth_gym.git
$ cd f1tenth_gym
$ git checkout exp_py
$ pip3 install --user -e gym/
```

**To run the LMPC algorithm**
1. To run the PID controller to generate safe set (the safe set is already generated in the repository, o you can run the LMPC directly):
```bash
cd planner
python3 mainPID.py
```
To run the LMPC algorithm
```bash
python3 main.py
```

## Relevant Folder Structure

1. `planner`: Contains the files for PID controller and LMPC controller
2. `gym`: The F1TENTH Gym Simulator files
3. `debug`: Files used for testing various componenets of the planner


## Files in \planner
| File | Description |
|----|----|
config_IMS_map.yaml   | The configuration data such as starting pose, waypoints file path, etc.
IMS_map.yaml | Map properties like image base, origin, etc
IMS_map_waypoints.csv | Contains the waypoints, distance from boundaries, and curvature of the center line of the track
initControllerParameters.py | Class to initialize the LMPC parameters
main.py | The main file of the planner
mainPID.py | The main file of the PID controller
PredictiveControllers.py | File which contains the LMPC class, which contains all the functions in the LMPC algorithm
PredictiveModel.py | This file contains the implementation for determnation of the convex local set, and is used to compute the ATV dynamics
PID_states.npz | Numpy file which contains the PID controller determined state and input values

## Demo on IMS racetrack
![Refer to this Google Drive link](https://drive.google.com/file/d/1OYTzROvC-OkJf8rAm6VVvPmjtx9mfA9g/view?usp=sharing)

<!-- ## Citing
If you find this Gym environment useful, please consider citing:

```
@inproceedings{okelly2020f1tenth,
  title={F1TENTH: An Open-source Evaluation Environment for Continuous Control and Reinforcement Learning},
  author={O’Kelly, Matthew and Zheng, Hongrui and Karthik, Dhruv and Mangharam, Rahul},
  booktitle={NeurIPS 2019 Competition and Demonstration Track},
  pages={77--89},
  year={2020},
  organization={PMLR}
}
``` -->
