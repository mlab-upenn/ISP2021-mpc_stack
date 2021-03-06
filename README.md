# ISP2021-mpc_stack
This is the github project for the F1Tenth Independent Study Projects 2021. In this project we are focusing on the development of a an MPC stack for the F1TENTH car.

The first controller of the stack is Hierarchical Model Predictive Control (HMPC) which has a high-level planner to generate a reference trajectory and uses a low-level MPC to optimall drive the car to track that trajectory. The high-level planner uses the idea of capturing the forthcoming track contour to estimate how much the car needs to re-orient itself in the future such that it would stay parallel to the track. Since there is a direct relationship between steering angle and the car's change in orientation, the controller can thus compute the appropriate steering angle for the car given the track configuration ahead. Additionally, because the controller knows the track contour ahead, it can detect sharp corners and, from there, control the car speed appropriately. After the appropriate steering angle and speed have been determined, they are then fixed to generate a reference trajectory that can optimally be tracked using the low-level MPC 

## Requirements
- Linux Ubuntu 
- Python 3
- F1tenth gym 

## F1tenth Gym Installation
`git clone https://github.com/f1tenth/f1tenth_gym.git`

`cd f1tenth_gym`

`git checkout exp_py`

`pip3 install --user -e gym`

More details on the gym installation can be found [here](https://f1tenth-gym.readthedocs.io/en/latest/index.html)

## Running the code - HMPC
* `Step 1:` Download the HMPC.py
* `Step 2:` Put HMPC.py into the `example` directory of the gym folder
* `Step 3:` The controller can be run by:

`cd examples`

`python3 HMPC.py`



