#!/bin/bash

File1=$1
echo "$File1"

File2=$2
echo "$File2"

File3=$3
echo "$File3"

File4=$4
echo "$File4"


awk -F":" '/latitude:/{print $2}' "$File1" > lat.txt
awk -F":" '/longitude:/{print $2}' "$File1" > lon.txt
awk -F":" '/altitude:/{print $2}' "$File1" > alt.txt


awk -F":" '/heading:/{print $2}' "$File2" > heading.txt

paste -d ' ' lat.txt lon.txt alt.txt heading.txt > GPS_data.txt

awk -F":" '/z:/{print $2}' "$File3" > x_imu.txt

awk 'NR % 3 ==2' x_imu.txt > yaw_vel.txt

awk -F":" '/x:/{print $2}' "$File4" > x_vel.txt

awk 'NR % 4 == 3' x_vel.txt > vel.txt

rm -f lat.txt lon.txt alt.txt heading.txt x_imu.txt x_vel.txt


