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
    def __init__(self, interface, port=1717, name='listener'):
        """
            CmdListener constructor
            Creates the network socket and initialises it to the appropriate port.
            It then binds the socket to a port and interface and wait for clients 
            to connect to it.
            Parameters:
                interface   -   The address of the interface that we wish to receive
                                connections on. Empty string ('') for all.
                port        -   The port number on the local/remote host to receive/transmit
                                data on. Defaults to an unused port 1717.
                name        -   The name of the thread
        """

        self.data_listeners = []
        self.dc_listeners = []

        af_inet_addr = (interface, port)

        try:
            #Create blocking (default) TCP socket
            self.sock_fd = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock_fd.settimeout(None)
            self.sock_fd.bind(af_inet_addr)

            #Allow connections
            self.sock_fd.listen(1)

        except socket.timeout, t:
            raise ConnectionException("Could not connect socket")
        except Exception, e:
            raise NetworkException("Could not initialise socket")

        try:
            #Create thread to frequently poll socket
            self.run_listener = True
            self.listener_name = name
            self.listener_thread = threading.Thread(target=self.listen, name=self.listener_name)
            self.listener_thread.start()
        except threading.ThreadError, t:
            raise CmdListenerException("Could not start new listener thread")

    def __del__(self):
        self.shutdown()

    def shutdown(self):
        self.run_listener = False
        self.listener_thread.join()
        self.sock_fd.close()

    def listen(self):
        #Wait for connection
        self.recv_sock, self.remote_addr = self.sock_fd.accept()
        print "Accepted connection!"

        #Update observers while running
        while self.run_listener:
            first_byte = self.recv_sock.recv(1)

            #Test whether client has dropped 
            if not first_byte:
                self.run_listener = False
                #Inform observers
                for callback in self.dc_listeners:
                    callback()

            else:
                msg_len = ord(first_byte)

                #Receive until we have an entire message
                msg_body = self.recv_sock.recv(msg_len)
                while len(msg_body) < msg_len:
                    msg_body = msg_body + self.recv_sock.recv(msg_len - len(msg_body))

                msg_parsed = msg.Msg(msg_body)

                #Give message to all registered observers
                for callback in self.data_listeners:
                    callback(msg_parsed)


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



class ConnectionException(Exception):
    def __init__(self, value):
        self.value = "Connection timeout: " + str(value)
    def __str__(self):
        return repr(self.value)

class NetworkException(Exception):
    def __init__(self, value):
        self.value = "Network fault: " + str(value)
    def __str__(self):
        return repr(self.value)

class CmdListenerException(Exception):
    def __init__(self, value):
        self.value = "Command Listener: " + str(value)
    def __str__(self):
        return repr(self.value)




#A small test harness for this file
def print_msg(msg):
    print str(msg)


#Basic module test code
if __name__ == '__main__':
    lis = CmdListener('')
    lis.add_data_listener(print_msg)
