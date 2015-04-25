#! /usr/bin/env python
#
# cmd_listener.py
# This module contains a class that can run on the
# robot and listen for connections.
# The robot creates a new thread every time that listen
# is called. Any data that is received is sent to every
# callback provided update_freq number of times per second.
#
# A new thread is created to continually poll for new data.
#
# The network aspect utilises TCP to send reliable messages over
# the network. This may change in future revisions.
# We are also more or less assuming IPv4 right now.
#
# This class is intended to support ROS messages but that may
# be reserved for a later revision.
#
# At the current time, the observers given to an object of this
# class are *NOT* started in a new thread. This means that the
# observers should take very special care to minimise the amount
# of processing that occurs in the callback method.
#
# The network protocol is currently as follows: A message header
# takes up the first byte of any message and contains the number
# of bytes in the message (not including the header byte). The rest
# of the message is parsed by its respective message class and so its
# format is not required here.



import sys
import socket
import time
import threading

import msg


class CmdListener:
    def __init__(self, addr, port=1717, name='listener'):
        """
            CmdListener constructor
            Creates the network socket and initialises it to the appropriate port.
            It then connects the socket so that it is ready to send *and* receive
            data.
            Parameters:
                addr        -   The IPv4 address to connect to as a string.
                port        -   The port number on the local/remote host to receive/transmit
                                data on. Defaults to an unused port 1717.
                name        -   The name of the thread
        """
        af_inet_addr = (addr, port)

        self.data_listeners = []
        self.dc_listeners = []

        try:
            #socket.create_connection always creates a TCP socket
            self.sock_fd = socket.create_connection(af_inet_addr)

            #Need to block when there is no data
            self.sock_fd.setblocking(1)
        except socket.timeout, t:
            raise ConnectionInception("Connection timed out for " + addr)
        except Exception, e:
            raise NetworkInception("Could not initialise socket for " + addr)

        try:
            #Create thread to frequently poll socket
            self.run_listener = True
            self.listener_name = name
            self.listener_thread = threading.Thread(target=self.listen, name=self.listener_name)
            self.listener_thread.start()
        except threading.ThreadError, t:
            raise CmdListenerInception("Could not start new listener thread")

    def __del__(self):
        self.shutdown()

    def shutdown(self):
        self.run_listener = False
        self.listener_thread.join()
        self.sock_fd.close()

    def listen(self):
        while self.run_listener:
            first_byte = self.sock_fd.recv(1)

            #Test whether client has dropped 
            if not first_byte:
                self.run_listener = False
                #Inform observers
                for callback in self.dc_listeners:
                    callback()

            else:
                msg_len = ord(first_byte)
                msg_body = msg.Msg(self.sock_fd.recv(msg_len))

                #Give message to all registered observers
                for callback in self.data_listeners:
                    callback(msg_body)


    def add_data_listener(self, callback):
        if callback not in self.data_listeners:
            self.data_listeners.append(callback)

    def rem_data_listener(self, callback):
        try:
            self.data_listeners.remove(callback)
        except:
            pass

    def add_dc_listener(self, callback):
        if callback not in self.dc_listeners:
            self.dc_listeners.append(callback)

    def rem_dc_listener(self, callback):
        try:
            self.dc_listeners.remove(callback)
        except:
            pass



Inception = Exception
class ConnectionInception(Inception):
    def __init__(self, value):
        self.value = "Connection timeout: " + str(value)
    def __str__(self):
        return repr(self.value)

class NetworkInception(Inception):
    def __init__(self, value):
        self.value = "Network fault: " + str(value)
    def __str__(self):
        return repr(self.value)

class CmdListenerInception(Inception):
    def __init__(self, value):
        self.value = "Command Listener: " + str(value)
    def __str__(self):
        return repr(self.value)

def print_msg(msg):
    print str(msg)


#Basic module test code
if __name__ == '__main__':
    lis = CmdListener('127.0.0.1')
    lis.add_data_listener(print_msg)
    time.sleep(10)
    lis.shutdown()

