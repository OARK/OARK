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

from dynamixel_msgs.msg import MotorState
import threading



#This is highly coupled to the class below. Right now
#the arguments must be passed in as text.
#Remarks: I'm not sure how much I like this approach
#and we may have to change it later on.
def __thread_getter(mutex_name, cond_func_name):
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

            @__thread_getter('thread_mutex', 'is_data_init')
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
        self.motor_state = None
        self.motor_mutex = threading.Lock()
        proxy.set_state_callback(controller_name, self._state_callback)

    def get_name(self):
        return self.controller_name

    def get_motor_state(self):
        raise NotImplementedError

    def command(self, value):
        raise NotImplementedError

    #Must be thread-safe
    def _state_callback(self, state):
        with self.motor_mutex:
            self.motor_state = state

    def is_motorstate_init(self):
        return self.motor_state != None

    @__thread_getter('motor_mutex', 'is_motorstate_init')
    def get_motor_id(self):
        return self.motor_state.id

    @__thread_getter('motor_mutex', 'is_motorstate_init')
    def get_motor_goal(self):
        return self.motor_state.goal
            
    @__thread_getter('motor_mutex', 'is_motorstate_init')
    def get_motor_position(self):
        return self.motor_state.position

    @__thread_getter('motor_mutex', 'is_motorstate_init')
    def get_motor_error(self):
        return self.motor_state.error

    @__thread_getter('motor_mutex', 'is_motorstate_init')
    def get_motor_speed(self):
        return self.motor_state.speed

    @__thread_getter('motor_mutex', 'is_motorstate_init')
    def get_motor_load(self):
        return self.motor_state.load

    @__thread_getter('motor_mutex', 'is_motorstate_init')
    def get_motor_voltage(self):
        return self.motor_state.voltage

    @__thread_getter('motor_mutex', 'is_motorstate_init')
    def get_motor_temp(self):
        return self.motor_state.temperature

    @__thread_getter('motor_mutex', 'is_motorstate_init')
    def is_motor_moving(self):
        return self.motor_state.moving
