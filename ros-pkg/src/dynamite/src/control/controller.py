#! /usr/bin/env python
#
# Contains a class to represent a joint
# controller.
#
# This class more or less just defines the common interface
# for the two subclasses: PosController and TorqueController.
#
# It is assumed that all relevant parameters have been pushed
# to the ROS parameter server. This can be changed quite easily
# in software at a later time by giving the option of specifying
# controllers completely.
#

from dynamixel_msgs.msg import JointState
import threading



#This is highly coupled to the class below. Right now
#the arguments must be passed in as text.
#Remarks: I'm not sure how much I like this approach
#and we may have to change it later on.
def thread_getter(mutex_name, cond_func_name):
    """
        This is a decorator that can be used to
        safely create getters for data that belongs
        to this object but is shared between multiple
        threads.

        arguments:
            mutex       - The mutex object that is needed before
                          access to the data. Needs to be
                          passed as a string.
            cond_func   - A function that, if false, will cause
                          None to be returned. This is because
                          if a volatile is supposed to be
                          initialised by another thread then it
                          is common to initialise that data to
                          None, so that needs to be tested for.
                          Needs to be passed as a string.

        Example:
            
            def is_data_init():
                return data != None

            @thread_getter('thread_mutex', 'is_data_init')
            def get_data():
                return data
        """
    def wrap(func):
        def wrap_getter(self, *args, **kwargs):
            mutex = getattr(self, mutex_name)
            with mutex:
                cond_func = getattr(self, cond_func_name)
                if cond_func(self):
                    return func(self, *args, **kwargs)
                else:
                    return None
        return wrap_getter
    return wrap


class Controller:
    def __init__(self, proxy, controller_name, port_ns):
        self.controller_name = controller_name
        self.port_ns = port_ns
        self.proxy = proxy

        #Ensure that state gets updated
        self.joint_state = None
        self.joint_mutex = threading.Lock()
        #self.proxy.set_state_callback(controller_name, self._state_callback)

    def get_name(self):
        return self.controller_name

    def get_joint_state(self):
        raise NotImplementedError

    def command(self, value):
        raise NotImplementedError('Command not implemented. Use subclass')

    #Must be thread-safe
    def _state_callback(self, state):
        with self.joint_mutex:
            self.joint_state = state

    def is_jointstate_init(self):
        return self.joint_state != None

    @thread_getter('joint_mutex', 'is_jointstate_init')
    def get_joint_ids(self):
        return self.joint_state.motor_ids

    @thread_getter('joint_mutex', 'is_jointstate_init')
    def get_joint_temps(self):
        return self.joint_state.motor_temps

    @thread_getter('joint_mutex', 'is_jointstate_init')
    def get_joint_goal(self):
        return self.joint_state.goal_pos
            
    @thread_getter('joint_mutex', 'is_jointstate_init')
    def get_joint_position(self):
        return self.joint_state.current_pos

    @thread_getter('joint_mutex', 'is_jointstate_init')
    def get_joint_error(self):
        return self.joint_state.error

    @thread_getter('joint_mutex', 'is_jointstate_init')
    def get_joint_velocity(self):
        return self.joint_state.velocity

    @thread_getter('joint_mutex', 'is_jointstate_init')
    def get_joint_load(self):
        return self.joint_state.load

    @thread_getter('joint_mutex', 'is_jointstate_init')
    def is_joint_moving(self):
        return self.joint_state.moving
