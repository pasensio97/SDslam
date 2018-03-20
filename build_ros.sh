#!/bin/bash

echo "Ruilding ROS nodes"

cd Examples/ROS/SD-SLAM
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j4
