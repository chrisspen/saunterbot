#!/bin/bash
# CS 2018-8-18
# Attempts to shift weight by lowering and raising the left leg.
# Make sure legs are spaced evenly apart, and haven't slide together.
set -e

cd "$(dirname "$0")"

# Ensure there's some tension on tendons.
echo "retract both"
printf "data: 1700\n---\n" | rostopic pub --once /torso_arduino/servo/knee/both/set std_msgs/Int16 > /dev/null
sleep 1

#while true; do 

    echo "retract left leg"
    printf "data: 1700\n---\n" | rostopic pub --once /torso_arduino/servo/knee/left/set std_msgs/Int16 > /dev/null
    sleep 0.15

    echo "extend left leg"
    printf "data: 2000\n---\n" | rostopic pub --once /torso_arduino/servo/knee/left/set std_msgs/Int16 > /dev/null
    sleep 0.25

#done
