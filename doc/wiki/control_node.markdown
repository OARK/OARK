# The Control Node #

#### Note ####
Note: 'controller' is used in more than one way in this article. 
*   It is used in the traditional software engineering sense to mean a part of
    the system that determines the control flow. 
*   It is also used in the robotics sense  to mean a model of a motor in
    software. A controller is called to control the motor.
#### End Note ####

The control node is a ROS node running on the Raspberry Pi. It is responsible
for controlling the other nodes and sending messages to the motors. The
control node consists of three main python modules:

*   control_node.py,
*   controller.py, and
*   manager.py.

These files are located at *ros-pkg/src/oark/src/control/* in the base of the
project.

## Interfaces ##

This code exports a number of interfaces that other nodes are allowed to use,
and it uses several interfaces of other code.

### Exported Interfaces ###

control_node.py exports three ROS services and one ROS topic. The three ROS
services are:

*   connected
    A service to call when a client connects to the robot. This is call by the
network node to inform the node of a connection. The address of the connected
client is passed to this service call. This service call allows the
control node to call a similar service on the video node. The video node can
then begin streaming video towards the client.
    The reason that the network node does not directly call the video node is
a little complicated. Essentially we want to controller of the nodes to be the
control node. The control node contains the settings for the robot (from the
configuration file), and so it should be the one to decide whether video will
be streamed at all or not.

*   disconnected
    A service to call when a client disconnects from the robot. This is called
by the network node to inform the node of a disconnect. This service allows
the control node to safely stop the robot and the video stream, if desired.

*   input request
    A service to call to request the list of inputs from the control node. The
control node loads the robot configuration from file, and the Android
controller requires the list of inputs from this configuration file. This
service can be called to retrieve those inputs. The network node calls this
service when it receives a network request for the inputs.


### Imported Interfaces ###

The control node mediates between different boundaries of the software.
Essentially it can be seen as a node that takes network messages and controls
motors. 

The control node calls services and uses topics from the dynamixel_motor
stack. It calls these services/topics to initialise and control motors.

The control node calls services on the video node to initialise the video. In
particular it calls the *start stream* and the *stop stream* services.


# Modules #

## control_node.py ##

This module is the controller class for the control node. It initialises the
node, and glues together the controllers and ROS services/topics.

## controllers.py ##

This module defines the types of controllers for AX12 motors. It defines a
torque controller, a position controller, and a generic parent controller.
These controllers should not be created anywhere except for inside of the
manager module. This is because the lifecycle of the controllers needs to be
managed. 

The dynamixel_motor stack has a similar implementation to the above. The
reason that this is duplicated is to make the robot easier to program. It is
much simpler to deal with objects than to deal with ROS RPC topics and
services. Annoyingly, this creates some redundancy, but it isn't unbearable
here.

## manager.py ##

This module defines a manager for the controllers. As mentioned above, the
lifecycle of the controllers needs to be managed and that is done here.
Creation of controllers and access to them should be performed through an
instance of this class.
