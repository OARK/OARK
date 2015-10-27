#!/usr/bin/env python

"""A basic network module to allow connections and pass messages
to any observers.
"""
import net_consts

import rospy
import socket
import threading

import msg


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
        self._send_mutex = threading.Lock()

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
        self._msg_sock.shutdown(socket.SHUT_RDWR)
        self._sock_fd.close()
        self._listener_thread.join()

    
    def _listen(self):
        while not self.stopped:
            #Wait for connection
            rospy.loginfo('Waiting for connection...')
            self._msg_sock, remote_addr = self._sock_fd.accept()
            with self._listener_mutex:
                for callback in self._conn_listeners:
                    callback(remote_addr)

            rospy.loginfo('Accepted connection from ' + str(remote_addr))
            
            #Start main message receiving/processing loop
            self.connected = True
            while self.connected:
                #Reliably read in message header
                msg_hdr = self._msg_sock.recv(net_consts.MSG_HEADER_LEN)
                while len(msg_hdr) < net_consts.MSG_HEADER_LEN and msg_hdr is not None:
                    new_bytes = self._msg_sock.recv(net_consts.MSG_HEADER_LEN - len(msg_hdr))
                    if new_bytes is None:
                        msg_hdr = None
                    else:
                        msg_hdr = msg_hdr + new_bytes


                #Test whether client has disconnected
                if not msg_hdr:
                    self.connected = False
                    #inform observers
                    with self._listener_mutex:
                        for callback in self._dc_listeners:
                            callback()
                else:
                    msg_type = ord(msg_hdr[0])

                    #Get message length from header
                    msg_len = 0
                    for i, byte in enumerate(reversed(msg_hdr[1:])):
                        msg_len = msg_len + (ord(byte) << (i * 8))

                    #Receive until we have an entire message
                    msg_body = self._msg_sock.recv(msg_len)
                    while len(msg_body) < msg_len:
                        msg_body = msg_body + self._msg_sock.recv(msg_len - len(msg_body))

                    #Inform observers
                    with self._listener_mutex:
                        for callback in self._data_listeners:
                            #Perform callback(msg_type, msg_body) on new thread
                            threading.Thread(target=callback, args=(msg_type, msg_body)).start()
                            



    def send(self, msg_type, data):
        """data should be a serialised *string*
        """
        with self._send_mutex:
            if not self.connected:
                raise NetworkException('Could not send data on socket. Not connected')

            #Calculate length of data in byte form. Done from lowest to highest byte
            msg_len = []
            for i in range(net_consts.MSG_HEADER_LEN-1):
                msg_len.append((len(data) >> (i*8)) % 256)
            #Correct order to be network byte order
            msg_len.reverse()

            print 'message length bytes: ', msg_len

            #Create message to send
            msg = chr(msg_type) + ''.join(map(chr, msg_len)) + data

            #Send message
            success = self._msg_sock.sendall(msg)
            if success != None:
                raise NetworkException('Could not send data on socket')


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
