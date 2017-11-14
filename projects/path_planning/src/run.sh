#!/bin/bash

echo "compiling ..."
cd ../build
make clean
make

cd ../src
echo "running ..."

start_lane = 1
max_acc    = 22.0
max_deacc  = 11.0
max_vel    = 50.0

echo "start_lane=" $p
echo "max_acc=" $i
echo "max_deacc=" $d
echo "max_vel=" $speed_goal

../build/path_planning $start_lane $max_acc $max_deacc $max_vel
