#!/bin/sh

echo "compiling ..."
g++ -std=c++11 main.cpp road.cpp vehicle.cpp cost.cpp -o run.out

chmod +x ./run.out
echo "running ..."
./run.out
