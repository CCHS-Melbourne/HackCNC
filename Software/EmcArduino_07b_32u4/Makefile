# Optional Makefile to compile/upload from outside the Arduino IDE
#
# If you're using the Arduino IDE program you can ignore this file.
#
#
#
# This Makefile assumes you're using the arduino & arduino-mk packages
# from Debian/Ubuntu but you can probably use others by updating the
# directory paths below.
#
# You may need to upload to the Leonardo board as root if your udev setup
# is weird like mine
#
# ie
# $ make
# $ sudo make upload
#
# (can override settings on command line, ie sudo ARDUINO_PORT=/dev/ttyACM3 make upload )
#
BOARD_TAG    = leonardo
ARDUINO_PORT ?= /dev/ttyACM0
ARDUINO_LIBS = digitalWriteFast NewLiquidCrystal Servo

ARDUINO_DIR ?= /usr/share/arduino
ARDMK_DIR     ?= /usr
AVR_TOOLS_DIR ?= /usr

# Arduino Leonardo custom reset command (NB: the echo on the end is important, due to parent makefile)
RESET_CMD = /bin/stty -F ${ARDUINO_PORT} 1200; sleep 0.5; echo

include ${ARDUINO_DIR}/Arduino.mk


