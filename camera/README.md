# Camera

Robots need to be remotely controlled, sometimes without the operator being able to see them. In this case, an on-board camera supplies video to the operator. This requires low latency video streaming from the device.

## Approach

### Notes: 2015-08-19
This approach is being updated, switching to using Python bindings for gstreamer, this will allow the camera to be migrated into the robot code as a sensor and should allow more on the fly control of parameters without needing to use bash scripts.

The Raspberry Pi has an optional camera module, it also has USB sockets that will allow the usage of a webcam. Current tests with the Raspberry Pi camera module have been unsuccessful, so the focus has been on a USB webcam. In particular, the Logitech C920 webcam, this webcam supports encoded H264 video from the camera directly, without the Raspberry Pi module needing to encode the stream.

Although the Raspberry Pi supports hardware encoding of h264, through OpenMax in it's GPU, tests have shown that using a webcam with native h264 support results in far lower CPU usage. Also, using the gstreamer framework, video streaming can be easily set up via the command line:

    gst-launch -v v4l2src device=/dev/video0 ! video/x-h264,width=800,height=600,framerate=10/1 ! h264parse ! rtph264pay pt=127 config-interval=4 ! udpsink host=<target ip> port=5000

However, the ribbon cable that is available with the Raspberry Pi camera module is better for routing. So this also has to be supported. Using the gstreamer framework, and the included raspivid program:

    raspivid -t 0 -h 480 -w 640 -fps 25 -hf -b 2000000 -o - | gst-launch-1.0 -v fdsrc ! video/x-h264,width=800,height=600,framerate=25/1 ! h264parse ! rtph264pay pt=127 config-interval=4 ! udpsink host=10.0.1.3 port=5000

Further tweaking of this command is needed since after awhile lag seems to build up.

On the target device, the gstreamer command line is:

    gst-launch-1.0 -v udpsrc port=5000 ! application/x-rtp, payload=127 ! rtph264depay ! avdec_h264 ! autovideosink
