#!/bin/bash
# CS 2018-8-18
# Shift weight by statically tilting with leg retraction,
# then see how quickly we fall once long leg is retracted.
#
# Leg measurements (from ground to hip):
#
# Leg length @ full extension = 22.5 cm
# Leg length @ full retraction = 13-13.5 cm
#
set -e

cd "$(dirname "$0")"

# Ensure there's some tension on tendons.
echo "retract both"
printf "data: 1800\n---\n" | rostopic pub --once /torso_arduino/servo/knee/both/set std_msgs/Int16 > /dev/null
sleep 1

#while true; do 

    echo "retract right leg to tilt right"
    printf "data: 1650\n---\n" | rostopic pub --once /torso_arduino/servo/knee/right/set std_msgs/Int16 > /dev/null
    sleep 2

    echo "retract left leg full to see how quickly we fall"
    printf "data: 1000\n---\n" | rostopic pub --once /torso_arduino/servo/knee/left/set std_msgs/Int16 > /dev/null
    sleep 0.25

#done
