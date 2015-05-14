# OpenCM Sketches #

This directory is intended to hold sketches that run on the Robotis OpenCM9.04 board.

## serial2_tosser.ino ##
This sketch is a slightly modified version of the DxlTosser example sketch that comes included with the Robotis OpenCM IDE v1.0.2. Essentially this sketch causes the OpenCM board to act like the USB2Dynamixel. It forwards messages from the Serial2 port to the Dynamixel bus and vice versa. Notably, the serial port must uses a lower baud rate because whatever is talking to it probably only supports lower baud rates.
