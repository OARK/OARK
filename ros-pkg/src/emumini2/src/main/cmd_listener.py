#!/usr/bin/env python

"""A basic network module to allow connections and pass messages
to any observers.
"""

import rospy
import socket
import threading


class CmdListener:
    def __init__(self, interface, port=1717):
        """Creates the network socket and initialises it to 
        the appropriate port. It then binds the socket to 
        a port and interface and wait for clients to connect to it.
        Parameters:
            interface - The address of the interface that we 
                        wish to receive connections on.
                        Empty string ('') for all.
            port      - The port number on the local/remote host
                        to receive/transmit data on. 
                        Defaults to an unused port 1717.
        """
        self._conn_listeners = []
        self._data_listeners = []
        self._dc_listeners = []
        self._listener_mutex = threading.Lock()

        af_inet_addr = (interface, port)

        try:
            #Create blocking (default) TCP socket
            self._sock_fd = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock_fd.settimeout(None)
            self._sock_fd.bind(af_inet_addr)

            #Allow connections
            self._sock_fd.listen(1)

        except socket.timeout, t:
            raise ConnectionException("Could not connect socket")
        except Exception, e:
            raise NetworkException("Could not initialise socket")

        rospy.loginfo('Socket initialised. Starting listener thread...')

        #stopped should be read-only
        self.stopped = False
        self._listener_thread = threading.Thread(target=self._listen)
        self._listener_thread.start()


    def shutdown(self):
        #Terminate socket and then the thread
        self.stopped = True
        self._sock_fd.close()
        self._listener_thread.join()

    
    def _listen(self):
        while not self.stopped:
            #Wait for connection
            recv_sock, remote_addr = self.sock_fd.accept()
            with self._listener_mutex:
                for callback in self._conn_listeners:
                    callback(remote_addr)

            rospy.loginfo('Accepted connection from ' + remote_addr)
            
            #Start main message receiving/processing loop
            connected = True
            while connected:
                first_byte = self.recv_sock.recv(1)

                #Test whether client has disconnected
                if not first_byte:
                    connect = False
                    #inform observers
                    with self._listener_mutex:
                        for callback in self._dc_listeners:
                            callback()
                else:
                    #Process message
                    msg_len = ord(first_byte)

                    #Receive until we have an entire message
                    msg_body = self.recv_sock.recv(msg_len)
                    while len(msg_body) < msg_len:
                        msg_body = msg_body + recv_sock.recv(msg_len - len(msg_body))

                    msg_parsed = msg.Msg(msg_body)
                    #Inform observers
                    with self._listener_mutex:
                        for callback in self._data_listeners:
                            callback(msg_parsed)


        def add_connect_listener(self, callback):
            if callback not in self._conn_listeners:
                with self._listener_mutex:
                    self._conn_listeners.append(callback)


        def add_data_listener(self, callback):
            if callback not in self._data_listeners:
                with self._listener_mutex:
                    self._data_listeners.append(callback)


        def add_dc_listener(self, callback):
            if callback not in self._dc_listeners:
                with self._listener_mutex:
                    self._dc_listeners.append(callback)



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
