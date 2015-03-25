# Software Requirements Specification

Open Framework for Controlling Low Cost 3D Robots \\
Software Engineering Project at Curtin University \\
Raymond Sheh, Hannes Herrmann, Mike Aldred, Delan Azabani, Tim Peskett

## Parties involved

**Client** \\
Dr Raymond Sheh

**Supervisor** \\
Dr Hannes Herrmann

**Team** \\
2015 #17

Mike Aldred \\
Delan Azabani \\
Tim Peskett

## Introduction

### Purpose

The purpose of this document is to define and agree upon a set of requirements
to be satisfied, goals to be met, and standards that should be upheld during
the development of the Open Framework for Controlling Low Cost 3D Robots,
henceforth referred to as "Ophir" for brevity, a nickname which is inspired by
its similarity to a subsequence of the project name's initial letters.

### Scope

Ophir will consist of a complete suite of software required to allow high
school students with minimal programming experience to easily operate, control,
and develop software for robot platforms available via the Open Academic Robot
Kit project, where the aforementioned deliverables will include:

TODO

### Definitions

**Controller** \\
A human interface device which connects, directly or indirectly, to one or more
Robots, providing a user interface through which control can be exerted, and
feedback collected. Controllers may mimic Peripherals, such as the 5DOF arm
controller [^1], rely solely on commodity hardware such as the Nvidia Shield
Portable, or combine the two approaches.

**OARK** \\
The Open Academic Robot Kit.

**Ophir** \\
This project.

**Peripheral** \\
An optional component of a Robot, which provides a service such as perception,
mobility, or dexterity. One example of a peripheral is the 5DOF robot arm [^2].

**Platform** \\
The foundation of a Robot, to which a Supervisor and Peripherals may be
attached. One example of such a foundation is the Six-Wheeled Wonder [^3].

**Robot** \\
A system that consists of a Platform, a Supervisor, and zero or more
Peripherals, controlled autonomously and/or by zero or more Controllers.

**Supervisor** \\
The central component of a Robot, comprising both hardware and software, and
exposing a common interface through which all queries and instructions to the
Robot must be conducted.

### Overview

The Open Academic Robot Kit provides hobbyists and researchers with an open,
extensible, independent standard for robot parts, specifications, and software.
By using the OARK, parts can be printed by users more durably than what is
offered by commercial robotics kits, without being locked into a vendor.

The overarching goal of this project is to create a software framework that
allows students who have minimal experience with programming to create their
own controllers for as wide a variety of robot configurations as possible.

TODO

## Technical investigations

### Fundamental components

### Platforms

### Supervisor hardware

### Supervisor middleware

### Supervisor software

### Controller hardware

### Controller software

## Requirements

### Stakeholders

### Functional requirements

### Non-functional requirements

### Constraints

### Validation of requirements

## Design

### Supervisors

### Controllers

### Supported platforms

### Supported peripherals

## Appendices

## References

[^1]:	Sheh, R. (2014). *Open Academic Robot Kit: 1DOF and 2DOF Joysticks and
	5DOF Arm Controller*. Retrieved from Thingiverse:
	<http://www.thingiverse.com/thing:370752>

[^2]:	Sheh, R. (2014). *Open Academic Robot Kit: 5 Degree of Freedom robot
	arm using Dynamixel AX-12A servos*. Retrieved from Thingiverse:
	<http://www.thingiverse.com/thing:368804>

[^3]:	Sheh, R. (2014). *Open Academic Robot Kit: The Six-Wheeled Wonder — a 6
	Wheel Drive robot platform using Dynamixel AX-12A servos*. Retrieved
	from Thingiverse: <http://www.thingiverse.com/thing:327689>
