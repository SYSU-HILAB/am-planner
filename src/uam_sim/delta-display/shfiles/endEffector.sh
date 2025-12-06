#!/bin/bash

param1=${1:--0.05}
param2=${2:-0.05}
param3=${3:--0.20}

param1_float=$(printf "%.5f" $param1)
param2_float=$(printf "%.5f" $param2)
param3_float=$(printf "%.5f" $param3)

rostopic pub -r 1 /body/end_effector geometry_msgs/Point "{x: $param1_float, y: $param2_float, z: $param3_float}"


