#!/bin/bash

rm -rf /home/odroid/Documents/Anastasis/Onboard-SDK-3.3/build

mkdir /home/odroid/Documents/Anastasis/Onboard-SDK-3.3/build

cmake -H/home/odroid/Documents/Anastasis/Onboard-SDK-3.3/ -B/home/odroid/Documents/Anastasis/Onboard-SDK-3.3/build  -DUSE_PRECISION_MISSIONS=OFF -DUSE_COLLISION_AVOIDANCE=OFF -DUSE_POINTCLOUD2LAS=OFF 

make -C /home/odroid/Documents/Anastasis/Onboard-SDK-3.3/build

cp /home/odroid/Documents/Anastasis/Onboard-SDK-3.3/UserConfig.txt /home/odroid/Documents/Anastasis/Onboard-SDK-3.3/build/bin
