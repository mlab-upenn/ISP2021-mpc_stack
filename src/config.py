n_states = 10                                     # Number of states
n_inputs = 3                                      # Number of inputs
n_states_inputs = n_states + n_inputs
sampling_time = 0.04                              # Sampling time, should be equal to the controller time
track_scale_factor = 0.8                          # Track width for the halfspace constraints
no_of_stages = 20                                 # Number of stages for the optimization
mixing = 0
no_of_polytopic_constraints = 2                   # Track halfspace constraints