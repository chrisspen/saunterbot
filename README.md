Saunterbot
==========

<img src="docs/img/bot/banner.png?raw=true" />

This is a small ROS-based platform for experimenting with dynamic biped balancing algorithms.

The image on the left is a CAD mock-up of the planned autonomous biped walker. On the right is the implemented version, mounted in a harness for testing its balancing ability.

An Arduino Mega is used for on-board actuation of servos and reading an MPU6050 accelerometer. It communicates to an external computer running ROS via USB for debugging and analyzing sensor output.

Once the basic balancing algorithm has been refined, the external computer will be replaced with a small single-board computer, like the Raspberry Pi, mounted in the compartment adjacent to the Arduino to make the platform fully autonomous.
