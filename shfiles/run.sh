#!/bin/bash
source ./devel/setup.bash

# Run one of the following simulation tasks by uncommenting the corresponding line.
# Only one task should be launched at a time.

# Grasp task
roslaunch plan_manage run_in_sim_grasp.launch

# Strike task
# roslaunch plan_manage run_in_sim_strike.launch

# Other task
# roslaunch plan_manage run_in_sim_other.launch