# Open Academic Robot Kit Software #

The Open Academic Robot Kit is an ambitious endeavour. It brings together lots
of software and hardware at once. This means that the software to operate OARK
robots is small and spread out rather than monolithic. After getting an
initial overview of the system, it is easiest to understand the workings of
the OARK system by comprehending each component at a time.

## Frameworks ##

To avoid reinventing the wheel, third party frameworks are used in places to
assist in the building of the OARK software. Usually, extensive knowledge of
these third party frameworks are not needed to work with the OARK software.
The following frameworks are noteworthy for their importance.

*   ROS

    The [Robot Operating System](http://www.ros.org/) provides a set of
packages for completing common robotics software tasks. Most importantly for
us, ROS provides [Remote Procedure
Call](https://en.wikipedia.org/wiki/Remote_procedure_call) functionality. This
allows us to write highly modular software where the modules can simply and
safely communicate with each other.
    In particular the dynamixel_motor package, created by Antons Rebguns, is
used to communicate with the AX12 motors.

*   Android

    Android is an open source operating system for smart phones, tablets, and
other embedded devices. The OARK uses the Android SDK to build an Android
application to control the Raspberry Pi software.


## The Components ##

### Terminology ###

Before going any further, the terminology used should be standardised to avoid
confusion. The best effort will be made in these documents to use the
following terms consistently and properly.

*   Android Software

    Refers to the software application operating on the Android device. This,
when packaged, comes in the form of an apk file.

*   Pi Software

    Refers to the software running on the Raspberry Pi.

*   OpenCM Software

    Refers to the software running on the OpenCM. The OpenCM code in this
particular case is a very short snippet, and only serves to relay messages to
the motors.

*   Component

    Refers to a large part of the system. In this case, a component will be
either the Android Software, the Pi Software, or the OpenCM Software depending
on context. Component is more or less interchangeable with device for our
purposes.

*   Network Code/Interface

    Refers to any software that provides communication over a network. In this
particular case, there is network code running on both the Android device and
the Pi to allow communication between them.

*   Node

    Refers to a ROS node. Care will be taken in these documents to only use `node' in
this manner. A ROS node can be thought of as the same thing as a computer
process. There are several ROS nodes (processes) running in parallel to power
the OARK.

*   Package

    Refers to a collection of modules that all have some higher-level purpose.
Roughly corresponds to a directory worth of code.
    
*   Module

    Refers to a collection of classes that all serve some purpose. Roughly
corresponds to a file of code.

*   Class

    Refers to a collection of variables and methods that abstract away some
internal detail and provide a higher-level interface for a task. In Java,
there is only one class per module/file, but in many other languages this is
not the case.

### Overall Structure ###

![Overall Structure UML Diagram](overall_structure.png)

Note: Almost all of the links in the above diagram are bidirectional. The
directions above indicate the main flow of data when sending commands from the
Android device to the motors. Information does tend to flow in both ways,
though.

The circles in the diagram represent boundaries between the different
components. For example, the Pi serial port determines where the Pi software
ends and the OpenCM software takes over. The distribution of detail in this
diagram does not represent the amount of code in that part of the system. For
the most part, an expanded view of the Pi software is shown because this is
the most important part for users to understand. Almost everything shown in
this diagram has a respective documentation page.

ROS has not been shown on this diagram, as it does not really have a place.
Showing ROS on the above diagram would be similar to trying to show Raspbian
on the diagram. The software is called in many places and all the other
software is essentially running on top of ROS.

One immediately noticeable peculiarity is the video node's direct connection
to the network interface/boundary. The video node does not utilise the network
node in sending its data. The sheer magnitude of data sent over this stream
would serve no purpose but to slow the system down. A more generic sensor
framework should be implemented that standardises nodes controlling their own
network access.
