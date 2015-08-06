#!/bin/bash
#
# Starts gstreamer

v4l2-ctl --set-ctrl=horizontal_flip=1
v4l2-ctl --set-ctrl=vertical_flip=1

gst-launch-1.0 -v v4l2src !  video/x-raw,width=640,height=480,framerate=25/1,color-rate=90000 ! omxh264enc control-rate=variable target-bitrate=1240000 ! video/x-h264 ! h264parse !  rtph264pay pt=96 config-interval=4 ! udpsink host=192.168.12.13 port=5000
