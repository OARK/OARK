= Programming the Emu Mini 2 =

The Emu Mini 2 has a companion Android app that can be used for control and for viewing the video feed. While this is great, it doesn't help us if we want to write our own programs for the Emu Mini 2.

This article details the steps to write a program for the Emu Mini 2 and then to get that program running. Keep in mind that the Emu Mini 2 is still in early days, and many significant changes are anticipated for the near future. As such, some of the content of this article will be subject to change over the next few weeks/months/years.


== How does the Emu Mini 2 software work? ==

The Emu Mini 2 software runs on top of a popular robotics architecture called //Robot Operating System// (ROS). ROS is not a simple piece of software, but one need not be an expert to be able to utilise ROS' most important features.

The most important things that ROS provides us with are;
  * existing robotics software, and
  * enhanced modularity.

ROS provides remote procedure call (RPC) functionality that allows robotics software to be written modularly. Each module of the software is called a //ROS node//. ROS nodes communicate using two different RPC mechanisms: topics, and services. The essential difference between these two mechanisms is that topics are (usually) intended for publishing data whereas services are intended as reliable inter-node method calls.

Don't worry if you only understood little of the last paragraph. We don't need to be familiar with ROS to get programming.


== How do I write a ROS node for the Emu Mini 2? ==

The existing software for the Emu Mini 2 saves us from writing software that interfaces directly with other ROS nodes. Essentially, we can just treat our ROS node as its own piece of software without worrying about the ROS ecosystem that is running.

The existing Emu Mini 2 software can be found at /opt/oark/ros-pkg/src/dynamite/src on the Raspbian Emu Mini 2 image. This software is very basic. Its purpose is to provide a simple set of objects for interacting with motors. If we wish to write our own code that uses these objects, we must place our new python module into this same directory.

Below is a piece of sample code that will move the robot forward for two seconds, and then move the arm to the home position. This same piece of sample code can be found inside of the aforementioned Emu Mini 2 software directory on the Raspbian image.

<code python>
#!/usr/bin/env python
#


"""A sample script for creating controllers for the emumini2 and driving
it around for a few seconds.
Written by Tim Peskett
"""

#Import ROS's python bindings
import rospy
import time

#Import our controller creation code
from control.pos_controller import PosController
from control.torque_controller import TorqueController
import control.manager_proxy as proxy





def init_wheels(man, port_ns):
    """Initialise all of the motors by creating a controllers. The
    names of each motor are currently set in a config file. The names
    shouldn't be duplicated here, but they are for the time being. This will
    be changed in the future.

    Returns a dictionary of controller objects.
    """
    wheels = dict()
    wheels['fl'] = TorqueController(man, 'wheel_fl', port_ns)
    wheels['fr'] = TorqueController(man, 'wheel_fr', port_ns)
    wheels['bl'] = TorqueController(man, 'wheel_bl', port_ns)
    wheels['br'] = TorqueController(man, 'wheel_br', port_ns)

    return wheels



def init_arm(man, port_ns):
    """Initialise all of the arm motors and create controllers for them.
    Again, the names of each motor are set elsewhere, and they must be used
    here verbatim for now.

    Returns a dictionary containing controllers for each part of the arm
    """
    arm = dict()
    arm['arm_base'] = PosController(man, 'arm_base', port_ns)
    arm['arm_wrist'] = PosController(man, 'arm_wrist', port_ns)
    arm['arm_hand'] = PosController(man, 'arm_hand', port_ns)

    return arm



if __name__ == '__main__':
    """The entry point for our script. We intialise all of the motors and then
    run them through a simple motion of moving forward for two seconds and then
    moving the arm.
    """

    #Serial port and dynamixel driver software constants
    port_ns = 'pi_out_port'
    manager_ns = 'dxl_manager'

    try:
        #Arbitrary node name for ROS to use
        rospy.init_node('em2_sample')

        #Wait until the dynamixel driver software is running (if it isn't already)
        rospy.wait_for_service('/%s/%s/start_controller'%(manager_ns, port_ns))

        #Initialise all of the motors
        man = proxy.ManagerProxy(manager_ns)
        wheels = init_wheels(man, port_ns)
        arm = init_arm(man, port_ns)
        
        #Start wheels moving forwards
        #One side must be moving clockwise and the other ccw
        #Range on torques is -7 to 7
        speed = 7.0
        wheels['fl'].set_torque(-speed)
        wheels['bl'].set_torque(-speed)
        wheels['fr'].set_torque(speed)
        wheels['br'].set_torque(speed)
        
        #Sleep while the robot moves forwards
        time.sleep(2)

        #Stop all wheels
        for wheel in wheels.values():
            wheel.stop()

        #Move all arm motors to resting point (lying flat)
        for arm_motor in arm.values():
            arm_motor.set_position(0)

        #Do nothing until we are sent the ROS shutdown message
        rospy.spin()

    except rospy.ROSInterruptException, r:
        print str(r)
        print 'ROSInterruptException occurred. Exiting...'
</code>

With a basic understanding of python, most of this code is rather simple. There are, however, a few things worth mentioning:
  * The module should be given execute permissions. This can be done with the unix utility `chmod`.
  * The purpose of rospy.init_node() is to tell ROS about our node so that we can perform remote procedure calls. The name that we supply here is arbitrary, but should be meaningful.
  * The purpose of rospy.spin() is simply to do nothing until ROS tells our node to shutdown. If we wish to have some sort of event loop then we can use `while rospy.is_shutdown():` instead.
  * As mentioned in the docstrings, the names for the motors (e.g wheel_fl) should not be changed. These are symbolic names that are assigned to the motors elsewhere.
  * The port_ns and manager_ns are simple constants that are necessary to interact with the dynamixel driver software. These should not need to be changed.
  * The value that needs to be sent to the arm to send it to home position is different depending on the configuration. Right now, the arm is configured to range from 0 (home) to somewhere around 3 (max extension). If the motor cannot safely go all the way to its furthest extent, then it is configured to stop early. This configuration can currently be changed with some difficulty, but probably shouldn't be lest the extents are set to some dangerous value. This will likely be made easier in future revisions of the software.


== How do I run my new ROS node? ==

Now that we have written our ROS node, we need to get it running under ROS. It is likely, if we have the correct Raspbian image, that there is already the default Emu Mini 2 control software running. We must first stop this software, and then start our own.

To list all of the currently running ROS nodes, we can type

<code bash>
rosnode list
</code>

and (for us, at least) it should return something akin to

<code bash>
/rosout
/em2_node
/dynamixels
</code>

The exact names are not set in stone, but they should be similar in effect to those above. If, instead, the command reports that the ROS core is not running, then try typing `roslaunch dynamite dynamite.launch` to get it running.

The ROS node that we want to kill is /em2_node. This node is currently controlling the Emu Mini 2. To kill this node we can type

<code bash>
rosnode kill /em2_node
</code>

and ROS should let us know that the node has been killed.

Now, to load our new node into ROS we can type 

<code bash>
rosrun dynamite sample.py
</code>

where sample.py is the name of our node. Remember that your code should be in the aforementioned /opt/oark/ros-pkg/src/dynamite/src directory before you run this command.

ROS should now begin to execute our node. It is likely that we will encounter some warning messages here about controllers starting up. These are none too serious.


== Known Bugs ==

  * Sometimes a motor will simply not start or stop when it is told. This seems to be a case of an operating system networking misconfiguration or some sort of inter-process race condition. The system has only recently been migrated to Raspbian and this is a bug that is yet to be ironed out.
  * When the new ROS node is started, the user is told that all of the controllers are already running. This isn't very pretty but it doesn't affect the execution of the software at all.


As an aside, it is well known that the Emu Mini 2 software currently has some design, usability, and stability issues. The software was written quickly and without much experience. This software will be rewritten to abide by better software engineering principles.
