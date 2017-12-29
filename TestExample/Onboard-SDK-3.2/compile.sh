#!/bin/bash

rm -rf /home/odroid/Documents/Anastasis/TestExample/Onboard-SDK-3.2/build

mkdir /home/odroid/Documents/Anastasis/TestExample/Onboard-SDK-3.2/build

cmake -H/home/odroid/Documents/Anastasis/TestExample/Onboard-SDK-3.2/ -B/home/odroid/Documents/Anastasis/TestExample/Onboard-SDK-3.2/build  -DUSE_PRECISION_MISSIONS=OFF -DUSE_COLLISION_AVOIDANCE=OFF -DUSE_POINTCLOUD2LAS=OFF

make -C /home/odroid/Documents/Anastasis/TestExample/Onboard-SDK-3.2/build/sample/Linux/Blocking/

cp /home/odroid/Documents/Anastasis/TestExample/UserConfig.txt /home/odroid/Documents/Anastasis/TestExample/Onboard-SDK-3.2/build/bin

cp /home/odroid/Documents/Anastasis/TestExample/UserConfig.txt /home/odroid/Documents/Anastasis/TestExample/Onboard-SDK-3.2/sample/Linux/Non-Blocking

cp /home/odroid/Documents/Anastasis/TestExample/UserConfig.txt /home/odroid/Documents/Anastasis/TestExample/Onboard-SDK-3.2/sample/Linux/Blocking
