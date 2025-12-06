#!/bin/bash

# 设置默认值
param1=${1:-85}
param2=${2:-10}
param3=${3:-50}

rostopic pub -1 /angles std_msgs/Float32MultiArray "layout:
  dim:
  - label: ''
    size: 3
    stride: 3
  data_offset: 0
data: [$param1, $param2, $param3]"

