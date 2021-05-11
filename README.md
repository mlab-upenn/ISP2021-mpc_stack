# MPCC

## Requirements
- Linux Ubuntu (tested on versions 20.04)
- ROS Noetic
- FORCESPRO
- F1tenth ROS simulator

## Installation
- FORCESPRO solver. Refer the documentation https://forces.embotech.com/Documentation/
- ![F1tenth simulator](https://github.com/f1tenth/f1tenth_simulator)


## Running the code
- Generate the solver file using mpcc.py file by uncommenting line number 75
- Run the launch file as
```
$ roslaunch mpcc mpcc.launch
```
- Run the MPCC Controller node
```
$ rosrun mpcc ros_mpcc.py
```


## Folder Structure

1. src contains the main scripts
2. data consists of the centerline files of the racetracks


## Files
| File | Description |
|----|----|
ros_mpcc.py   | The ROS NOde 
mpcc.py | Main file which runs the solver
model.py | Contains the dynamics of the model
cost.py | Contains the cost function
constraints.py | Halfspace track constraints
track.py | Spline fitting of the centerline
params.py | Contains Model, Cost and Constraints bounds
config.py | Number of stages and sampling time
structs.py | Struct objects such as State and Input
solver.py | Low level interface for the FORCESPRO solver
centerline_publisher.py | ROS node to publish the centerline marker msg
track_constraint_pub.py | ROS node to publish the halfspace track constraints marker msg

## Demo on IMS racetrack
![](https://github.com/mlab-upenn/ISP2021-mpc_stack/blob/mpcc/gifs/demo_mpcc.gif)
