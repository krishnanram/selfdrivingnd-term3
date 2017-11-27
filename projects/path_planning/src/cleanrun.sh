#!/bin/bash

echo "clean, compile and run ..."
cd ../build
make clean
cd ../src
pwd
./run.sh
