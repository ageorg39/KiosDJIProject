#!/bin/sh

#gets run automatically at startup by /etc/rc.local

sleep 20s

cp -n /home/odroid/Documents/Anastasis/Onboard-SDK-3.3/build/bin/position.txt /home/odroid/Documents/Anastasis/Onboard-SDK-3.3/FlightData/position1.txt


cd /home/odroid/Documents/Anastasis/Onboard-SDK-3.3/build/bin; ./djiosdk-LiDARControl-sample
