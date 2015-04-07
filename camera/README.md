# Camera

Robots need to be remotely controlled, sometimes without the operator being able to see them. In this case, an on-board camera supplies video to the operator. This requires low latency video streaming from the device.

## Approach

The Raspberry Pi has an optional camera module, it also has USB sockets that will allow the usage of a webcam. Current tests with the Raspberry Pi camera module have been unsuccessful, so the focus has been on a USB webcam. In particular, the Logitech C920 webcam, this webcam supports encoded H264 video from the camera directly, without the Raspberry Pi module needing to encode the stream.

Although the Raspberry Pi supports hardware encoding of h264, through OpenMax in it's GPU, tests have shown that using a webcam with native h264 support results in far lower CPU usage. Also, using the gstreamer framework, video streaming can be easily set up via the command line:

    gst-launch -v v4l2src device=/dev/video0 ! video/x-h264,width=800,height=600,framerate=10/1 ! h264parse ! rtp264pay pt=127 config-interval=4 ! udpsink host=<target ip> port=5000
