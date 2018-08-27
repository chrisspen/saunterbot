#!/bin/bash
# CS 2018-8-18
# Attempts to shift weight by sequentially lowering and raising each leg.
set -e

cd "$(dirname "$0")"

#1800,1800
echo "retract both"
printf "data: 1800\n---\n" | rostopic pub --once /torso_arduino/servo/knee/both/set std_msgs/Int16 > /dev/null
#sleep 1

#1800,1700
echo "retract right to tilt right"
printf "data: 1700\n---\n" | rostopic pub --once /torso_arduino/servo/knee/right/set std_msgs/Int16 > /dev/null
#sleep 3

#echo "extend left to tilt more right"
#printf "data: 1900\n---\n" | rostopic pub --once /torso_arduino/servo/knee/left/set std_msgs/Int16 > /dev/null
#sleep 0.1

#1600,1700
echo "retract left to lift leg"
printf "data: 1600\n---\n" | rostopic pub --once /torso_arduino/servo/knee/left/set std_msgs/Int16 > /dev/null
#sleep 3

#1600,1800
echo "extend right to rebalance and help left clear"
printf "data: 1800\n---\n" | rostopic pub --once /torso_arduino/servo/knee/left/set std_msgs/Int16 > /dev/null
#sleep 3

#1800,1800
echo "extend left to touch down and stabilize"
printf "data: 1800\n---\n" | rostopic pub --once /torso_arduino/servo/knee/left/set std_msgs/Int16 > /dev/null
#sleep 3

echo "extend both"
./relax_knees.sh
