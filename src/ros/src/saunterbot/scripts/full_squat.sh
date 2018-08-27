#!/bin/bash

#echo "Fully retracting left knee servo..."
#printf "data: 1000\n---\n" | rostopic pub --once /torso_arduino/servo/knee/left/set std_msgs/Int16 > /dev/null

#echo "Fully retracting right knee servo..."
#printf "data: 2000\n---\n" | rostopic pub --once /torso_arduino/servo/knee/right/set std_msgs/Int16 > /dev/null

echo "Fully retracting knee servos..."
printf "data: 1000\n---\n" | rostopic pub --once /torso_arduino/servo/knee/both/set std_msgs/Int16 > /dev/null

echo "Done."
