#!/bin/bash
# Initializes a shell, running on the robot, for ROS.
# Call by sourcing, e.g. source setup.bash
# http://wiki.ros.org/catkin/Tutorials/workspace_overlaying

_CURRENT_DIR=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)
_ROS_VERSION=kinetic

# Load our local settings.
if [ -f "$_CURRENT_DIR/setup_local.bash" ]; then
    . $_CURRENT_DIR/setup_local.bash
fi

# Load ROS's settings.
source /opt/ros/$_ROS_VERSION/setup.bash

# Activate our custom Python virtualenv.
if [ -f "$_CURRENT_DIR/../../.env/bin/activate" ]; then
    source $_CURRENT_DIR/../../.env/bin/activate
fi

# Load our overlay settings.
if [ -f "$_CURRENT_DIR/../overlay/devel/setup.bash" ]; then
    source $_CURRENT_DIR/../overlay/devel/setup.bash --extend
fi
source $_CURRENT_DIR/devel/setup.sh --extend

# Load our custom log settings.
#export ROS_PYTHON_LOG_CONFIG_FILE=$_CURRENT_DIR/src/ros_homebot/config/logging.conf

# Expose our custom Python executables.
#PATH=$PATH:$_CURRENT_DIR/src/ros_homebot_python/src/ros_homebot_python/bin
#export PATH
