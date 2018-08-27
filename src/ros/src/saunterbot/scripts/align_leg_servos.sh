#!/bin/bash
# CS 2018-8-18
# Aligns leg servos for mounting legs in their default positions.

echo "Centering hip servos..."
#rostopic pub --once /torso_arduino/servo/hip/set std_msgs/Int16 1500
printf "data: 1500\n---\n" | rostopic pub --once /torso_arduino/servo/hip/set std_msgs/Int16 > /dev/null

echo "Fully extending left knee servo..."
#left extended (position with tendon pointed at hip axle)
#rostopic pub --once /torso_arduino/servo/knee/left/set std_msgs/Int16 1950
printf "data: 2000\n---\n" | rostopic pub --once /torso_arduino/servo/knee/left/set std_msgs/Int16 > /dev/null
#left retracted
#rostopic pub --once /torso_arduino/servo/knee/left/set std_msgs/Int16 1050

echo "Fully extending right knee servo..."
#right extended (position with tendon pointed at hip axle)
#rostopic pub --once /torso_arduino/servo/knee/right/set std_msgs/Int16 1050
printf "data: 2000\n---\n" | rostopic pub --once /torso_arduino/servo/knee/right/set std_msgs/Int16 > /dev/null
#right retracted
#rostopic pub --once /torso_arduino/servo/knee/right/set std_msgs/Int16 1950

echo "Done."
