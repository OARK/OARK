# Raspberry Pi Software #

The Raspberry Pi is really the brain of an OARK robot. It runs the software
that controls the motors, listens on the network for connecting Android
devices, and streams video to connected hosts.

## The Environment ##

### Operating System ###

The OARK Raspberry Pi image runs [Raspbian](https://www.raspbian.org/).
Raspbian is a variation on Debian, a very popular Linux distribution for
desktops and other personal machines. The large amount of support available
for Raspbian makes it a good choice for the OARK environment.

A basic understanding of Linux is needed for anything but the most trivial of
OARK use-cases. At the very least, one should be able to edit files on Linux.
Experience with the Linux terminal is very useful.

### ROS ###

The Raspberry Pi software runs on top of a popular robotics architecture
called the *Robot Operating System* (ROS). ROS is not a simple piece of
software, but on need not be an expert to be able to utilise ROS's most
important features. The first thing to know about ROS is that it is not an
operating system in the same way that Windows, OSX, or a flavour of Linux is
an operating system. ROS is a bundle of software that runs on Linux to perform
common robotics tasks.

The most important things that ROS provides us with are;
* existing robotics software, and
* enhanced modularity.
The Raspberry Pi runs the [Robot Operating System (ROS)](http://www.ros.org/).

ROS provides remote procedure call (RPC) functionality that allows robotics
software to be written modularly. Each module of the software is called a
*ROS node*. ROS nodes communicate using two different RPC mechanisms:
topics, and services. The essential difference between these two mechanisms is
that topics are (usually) intended for publishing data whereas services are
intended as reliable inter-node method calls.

Don't worry if you only understood little of the last paragraph. We don't need
to be familiar with ROS to start programming.

### dynamixel_motor ###

[*dynamixel_motor*](http://wiki.ros.org/dynamixel_motor) is a ROS software
stack created by Antons Rebguns of the University of Arizona. dynamixel_motor
is used by the OARK software to control and manage the AX12 motors. An
understanding of dynamixel_motor is not necessary for understanding how to use
the OARK software.


## OARK Software Layout ##

The OARK Raspberry Pi Software is located at */opt/oark/ros-pkg/src/oark* on
the OARK Raspberry Pi image. The software is split into three main parts: the
control node, the video node, and the network node. Each of these nodes is
explained in more detail in its own article. Along with these, there is
extensive in-code documentation to assist in coming to grips with this code.

There is a dedicated documentation page for each of these modules.
