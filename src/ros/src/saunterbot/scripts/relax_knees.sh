#!/bin/bash
# CS 2018-8-18
# Fully extends legs, relaxing tendons.
printf "data: 2000\n---\n" | rostopic pub --once /torso_arduino/servo/knee/both/set std_msgs/Int16 > /dev/null
