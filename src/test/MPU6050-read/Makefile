# Arduino Make file. Refer to https://github.com/sudar/Arduino-Makefile

# Arduino < 1.5
BOARD_TAG     = mega2560

# Arduino >= 1.5
#BOARD_TAG    = mega
#BOARD_SUB    = atmega2560

MONITOR_PORT  = /dev/ttyACM0
AVRDUDE_OPTS = -v
ARDUINO_LIBS = Wire I2Cdev MPU6050
ARDUINO_SKETCHBOOK = $(realpath .)/lib
USER_LIB_PATH = $(realpath .)/lib
include /usr/share/arduino/Arduino.mk
