# The Video Node #


The video node is the simplest of the OARK ROS nodes. Its sole responsible is
to handle the starting and stopping of the video stream. Video streams over
UDP and uses gstreamer to handle the video pipeline.

The port/address that the video is streamed to can be changed in the
oark.launch roslaunch file.

The video node is located at *ros-pkg/src/oark/src/camera/* from the base of
the project directory.

## Interfaces ##

The video node uses gstreamer to actually perform the streaming. Luckily,
gstreamer has python bindings to allow us to control it programmatically.

### Exported Interfaces ###

The video node exports two services for other nodes to use:

*   start stream
    Takes an IP address. Begins streaming video towards a specified address.
*   stop stream
    Stops the streaming video.

These services are called by the control node when a client connects or
disconnects.

### Imported Interfaces ###

The gstreamer module makes heavy use of the python gstreamer bindings. These
can be seen by inspecting the module.

It uses these to initialise gstreamer and construct the pipeline.



## Modules ##

### video_node.py ##

The ROS video node controller. This module initialises the ROS node, and
manages the video stream. It loads a configuration file to specify the
gstreamer pipeline. This makes it very simple to change the gstreamer pipeline
at any time.


## Pipeline Configuration ##

At the current time, dynamic pipeline configuration is not supported. The
groundwork exists for this, but it is not implemented.

The pipeline is specified in a yaml file, the path of which is passed into the
node on startup. The pipeline inside of the video node directory is a very
simple pipeline that will operate correctly on the Raspberry Pi with a stock
standard Raspberry Pi camera module. The pipeline configuration file supports
defining elements, properties on those elements, and backup elements if
creating elements fails for some reason or other.
