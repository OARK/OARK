====== Software ======
The Open Academic Robot Kit software is currently being developed as part of a Software Engineering Project at Curtin University. Although it's a work in progress, the source code can be found on Github: [[https://github.com/OARK/OARK|OARK]].

The following sections outline the design of the software, development, and build processes to make it more accessible externally.

There are three parts to this software, the controller software, robot software, and software used to develop and build the other two parts.

===== Controller Software =====

The controller software is an Android application that is used to control the robot remotely. The Android platform was selected because it can be developed on a wide range of platforms, has a larger amount of devices available for it cheaply, and is considered a much more open platform.

Currently Android Studio is used for development, the root directory of the project being located under: ''android/EmuMini2/'', this directory can be opened in Android Studio via ''Open Project'' and allow for the project to be built.

===== Robot Software =====

The robot software is built on ROS, an open platform for developing robotics. ROS allows for a variety of languages to be used for development, but Python is its "native" language. So the Open Academic Robot kit is written in Python. This software is built as a ROS package and can be found under ''android/ros-pkg''

There are also various bits of software that aren't directly related to controlling the robot itself (the motors, etc), that are still run on the robot. These include the configuration tool that is used to change the wifi settings on the robot.

===== Development Software =====

In order to try and make the development process easier, and try to move towards a one-step build process the third major part of the software is a combination of scripts to build the robot software into a distributable image.

Details can be found on [[designs:software:development|development software]].
