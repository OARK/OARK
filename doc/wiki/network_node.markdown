# The Network Node #

The network node sits on the boundary of the system, and handles network
connections between the Raspberry Pi and the Android device. The network node
only supports one simultaneous connection (by design). When the client
disconnects, the node will go back to a listening state and will accept the
next connection.

This node listens on port 1717 by default, but this port can be changed very
easily in the oark.launch file.

The modules of the network node can be seen at *ros-pkg/src/oark/src/net/* in
the base of the project directory.


## Interfaces ##

The network node is all about interfacing with other software. It interfaces
with the Android device and the control node.

### Exported Interfaces ###

The network node listens on a socket on the specified port (default: 1717) and
fields connections and data. When the client connects, the network node begins
accepted messages in a certain format. The format of the messages are as
follows.

*   Byte 0: A message type byte. This is a unique integer identifier that
    corresponds to the ROS message type that the message contains. For
example, the value '1' means that the message is a 'Command' message.
*   Byte 1,2: The message length. These two bytes are a network-order
    (big-endian) encoding of the length of the *rest* of the message. This is
two bytes long to allow messages greater than 258 (header + 255) bytes in
length.
*   Bytes 3-?: The message data. This is a serialised ROS message. The data
    here will depend on the message type. This serialised data is passed to
the ROS message deserialise functions.

The messages sent by this node should be of the exact same form.


### Imported Interfaces ###

The network node communicates with the control node to actually perform
actions. It uses the following services/topics from the control node.

*   Command

    A topic that is published to when a Command message is received. This
publishes an array of float values to the control node.

*   Connected

    A service that is called when a client connects. The IP address of the
client is passed to this service call.

*   Disconnected

    A service that is called when a client disconnects.

*   InputRequest

    A service that is called when the Android device wishes to receive a list
of the inputs. See the control node documentation for more information on
this.


## Modules ##

### net_node.py ###

The controller class for the network node. Initialises a ROS node and then
defines several callbacks for the CmdListener class. These callbacks will be
called when the appropriate events occur and then this module will pass these
messages on as appropriate.


### cmd_listener.py ###

Handles the actual networking side of the node. Creates a socket and waits for
connections on that socket. When a client disconnects, the module will begin
listening for new connections.

Callbacks can be registered with the CmdListener class that allow the node to
be notified when events occur. These callbacks are registered in the
net_node.py module.
    

### net_consts.py ###

Contains some constants for this node. Things such as the message header
length and the mapping from message types to ROS message classes. This same
mapping needs to also occur on the Android device.


## Problems ##

Closing down the network node can be a difficult process. A simple way to kill
the socket is yet to be found. Usually, when ROS elevates its signals to
SIGKILL, the socket should close properly. If the socket is still in use but
the process is *not* still running, then waiting for a minute or two for the
operating system to release the resource may be required.
