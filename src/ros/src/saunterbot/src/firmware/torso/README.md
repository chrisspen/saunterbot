Build with:

    make && make upload

Note, it seems the current wiring configuration, that the Arduino Mega often will not respond to `make upload` when attached to only USB.

IT MUST BE POWERED EXTERNALLY while also being attached via USB.

Usage
-----

Launch ROS serial node:

    cd src/ros
    . ./setup.bash
    roslaunch saunterbot serial_torso.launch

In a separate terminal, setup a ROS interactive shell:

    cd src/ros
    . ./setup.bash
    rostopic list
    rosservice list

Manually set leg position with:

    rostopic pub --once /torso_arduino/servo/hip/set std_msgs/Int16 1500
