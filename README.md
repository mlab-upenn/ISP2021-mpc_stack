# Learning Model Predictive Controller (LMPC) on The F1TENTH Gym environment

This is the repository for the LMPC implementation on the F1TENTH Gym environment.

# Requirements
- Linux Ubuntu (tested on versions 20.04)
- Python 3.8.5
- F1TENTH Gym Simulator
- OSQP solver [(Install from here)](https://osqp.org/docs/get_started/sources.html)

The documentation of the simulation environment can be found [here](https://f1tenth-gym.readthedocs.io/en/latest/).

## Install the simulator using:
This algorithm uses a modified version of the Gym simulator

**You can install the environment by running**

```bash
$ pip3 install --user -e gym/
```
## Running the code

**To run the LMPC algorithm**
1. In step 1 enter the `planner` directory:
```bash
cd planner
```
2. In step 2, to run the PID controller, do the following (not compulsory to run to run LMPC)
```bash
python3 mainPID.py
```
3. In step 3, to run the LMPC algorithm, do the following
```bash
python3 main.py
```

## Relevant Folder Structure

1. `\planner`: Contains the files for PID controller and LMPC controller
2. `\gym`: The F1TENTH Gym Simulator files
3. `\planner\debug`: Files used for testing various componenets of the planner and pre-computation of path curvature


## Files in `\planner`
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

## Files in `\planner\debug`
| File | Description |
|----|----|
generate_curvature.py   | Script to pre-compute the curvatures at each waypoint and store it back into the csv file
curvature_test.py | Script to verify curvature values from two methods used in the LMPC algorithm. One uses the (X,Y) position while the other one computes it from distance along the center line

## Demo on IMS racetrack
Video sped up 5x to ensure the simulator frequency is similar to real world frequency 
<a href="http://www.youtube.com/watch?feature=player_embedded&v=S3I64Jzw--Y
" target="_blank"><img src="http://img.youtube.com/vi/S3I64Jzw--Y/0.jpg"></a>
