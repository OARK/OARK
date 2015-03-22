# SEP Open Academic Robotics Kit Functional Specification
Author: Mike Aldred

## About this specification
This specification is simply the starting point for the design of the OARK, it will change over time as it is not complete. Do not view it as something that is immutable and unchangeable, it should be a living document.

## Overview
The Open Academic Robotics Kit (OARK) is an initiative to make it easier for people to hack together their own robots. By hack, we don't mean throw something together haphazardly, but play, experiment, and importantly have fun making their own robots. Yes, there are kits that exist that allow people to build their own robots, but we want the OARK to be a platform that lets people with minimal knowledge and experience build _their_ own robot.

---
OARK enables people to experiment to build _their_ robot

---

The OARK does assume access to a 3D printer, and also access to equipment that can't be 3D printed, servos, wiring, controller boards and the like.

Here's a brief summary of how the OARK works for a user:

1. User visits the website and downloads the plans for an existing design.
2. 3D prints design using .stl that are available on site for that design.
3. Acquires necessary parts that can't be 3D printed.
4. Assembles the parts as per the instructions.
5. With the robot assembled, the user downloads the single OARK image for the robot controller board.
6. With the controller board booted and operational, the user uploads the description of the robot into the OARK software on the board.
7. User associates the robot with their controller device, either a Nvidia shield, or Android based device.
8. User can control the robot and generally have fun.

This is no different from most robotic kits that someone could get, however, the difference is when the user wants to customise, or build their own design from scratch. The software on the controller board, and the controller device, should not have to be changed. Changes will have to be performed, and some coding is unavoidable however it should be as minimal as possible.

## Major Features

* Relatively cheap, total cost for a robot should be in the sub $1000 range.
* Allow different designs to be controlled from the same controller.
* Programming new robot designs should not involve any knowledge of kinematics, or anything above basic ST151 level.

## Design Goals

All design and engineering decisions will be taken with the following principle in mind:

---
Simple is better than complicated

---

## Major Components
There are a few components to the overall concept of OARK.

### Physical Designs
The OARK is to offer some starting designs, this gives users something they can build and start with straight away, rather than having to make their own design from scratch. Currently the OARK has two designs available.

### Robot server
Software that runs on the embedded controller in the body of the robot, this is the brains of the robot. It will take the input from the controlling device and translate the intentions of the user into the corresponding actions of the robot.

### Robot controller
The user's interface to controlling the robot, for example the joystick and buttons on an Nvidia shield. This software is responsible for taking the user's intentions and forwarding them onto the robot server. However, it's not just a one-way communication, the robot server needs to communicate the capabilities of the robot to the controller so that meaningful mappings are possible.

## About the first release

The first release will be as simple as possible, given our inexperience in the area of robotics, we will be unsure of any necessary abstractions that will be necessary between the body of the robot and the translations necessary for the different methods of controlling it. At the moment the first release should be able to be used with existing robot designs without needing to reflash the image on the robot server, or the robot controller.

## Physical Architecture

Currently we are planning to use the Raspberry Pi 2, model B for the robot server, and a Nvidia shield as the robot controller.

## Software Architecture

## The User Experience

## Future Features
