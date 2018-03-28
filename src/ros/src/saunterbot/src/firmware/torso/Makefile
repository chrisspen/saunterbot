# Arduino Make file. Refer to https://github.com/sudar/Arduino-Makefile
#BOARD_TAG    = uno

# Arduino < 1.5
BOARD_TAG     = mega2560

# Arduino >= 1.5
#BOARD_TAG    = mega
#BOARD_SUB    = atmega2560

MONITOR_PORT  = /dev/ttyACM0
#BOARD_TAG    = uno_pro
#MONITOR_PORT = /dev/ttyACM0
#MONITOR_PORT = $(shell list_arduinos.py torso)
AVRDUDE_OPTS = -v
#ARDUINO_LIBS = ros_lib Wire
ARDUINO_LIBS = ros_lib Wire I2Cdev MPU6050
ARDUINO_SKETCHBOOK = $(realpath .)/lib
USER_LIB_PATH = $(realpath .)/lib
include /usr/share/arduino/Arduino.mk
