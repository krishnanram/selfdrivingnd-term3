#!/bin/bash

echo "compiling ..."
cd ../build
#make clean
#make

cd ../src
echo "running ..."

start_lane=1
max_acc=22.0
max_deacc=11.0
max_vel=50.0

echo "start_lane=" $start_lane
echo "max_acc=" $max_acc
echo "max_deacc=" $max_deacc
echo "max_vel=" $max_vel

../build/path_planning $start_lane $max_acc $max_deacc $max_vel
