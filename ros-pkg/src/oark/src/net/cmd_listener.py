#!/usr/bin/env python

"""A basic network module to allow connections and pass messages
to any observers.
The message sent over the network are of the following form.

            Byte#       Value               Meaning
Msg Type    0           0 - NUM_MESSAGES    A unique integer id for the message
Msg Len     1-2         0 - 65535           The length of Msg Data.
Msg Data    3-MsgLen+3  Array of bytes      The message data.

The number of bytes in Msg Len can be changed in net_consts. In this case,
the offset of the Msg Data will simply move up or down by however many bytes
it is changed. 
See net_consts.py in this package for the available message types.
"""

import rospy
import socket
import threading

import net_consts


class CmdListener:
    """A class to create a socket, listen on it, and then relay
    any data received on that socket. It also has functionality
    to send data over the same socket. The CmdListener only allows
    one connection at a time.
    """

    def __init__(self, interface, port):
        """Creates the network socket and initialises it to 
        the appropriate port. It then binds the socket to 
        a port and interface and wait for clients to connect to it.
        Parameters:
            interface - The address of the interface that we 
                        wish to receive connections on.
                        Empty string ('') for all.
            port      - The port number on the local/remote host
                        to receive/transmit data on. 
        """
        self._conn_listeners = []
        self._data_listeners = []
        self._dc_listeners = []

        #Mutexes to ensure synchronised sending and atomic access to the
        #lists of listeners
        self._listener_mutex = threading.Lock()
        self._send_mutex = threading.Lock()

        #Serialise address data for usage with sockets
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
        
        #A flag set to true when the caller wishes for this CmdListener to
        #close its connection and shutdown. Shutting down sockets properly
        #is like pulling teeth.
        self._stopped = False

        #Start listening in a new thread.
        self._listener_thread = threading.Thread(target=self._listen)
        self._listener_thread.start()


    def shutdown(self):
        """Shutdown the CmdListener object. The CmdListener object should
        not be used after this method has been called.
        """
        #Terminate socket and then the thread
        self._stopped = True
        self._msg_sock.shutdown(socket.SHUT_RDWR)
        self._sock_fd.close()
        self._listener_thread.join()

    
    def _listen(self):
        """Listens on a socket and relays any received messages
        on new threads.
        """
        while not self._stopped:
            #Wait for connection
            rospy.loginfo('Waiting for connection...')
            self._msg_sock, remote_addr = self._sock_fd.accept()
            with self._listener_mutex:
                for callback in self._conn_listeners:
                    callback(remote_addr)

            rospy.loginfo('Accepted connection from ' + str(remote_addr))
            
            #Client connected. Start event processing loop
            self.connected = True
            while self.connected:
                #Reliably read in message header
                #This method will block!
                msg_hdr = self._recv_reliable(self._msg_sock, net_consts.MSG_HEADER_LEN)

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
                    msg_body = self._recv_reliable(self._msg_sock, msg_len)

                    #Inform observers
                    with self._listener_mutex:
                        for callback in self._data_listeners:
                            #Perform callback(msg_type, msg_body) on new thread
                            threading.Thread(target=callback, args=(msg_type, msg_body)).start()
                            

    def _recv_reliable(self, socket, length):
        """Recv length number of bytes on socket. Usually recvs return as soon as
        there is data. We want to receive the right amount of data.
        Params:
            socket - The socket to recv on.
            length - The number of bytes that we wish to receive.

        Returns a list of bytes received, or None if client disconnects.
        """
        data = socket.recv(length)
        while data and len(data) < length:
            new_bytes = socket.recv(length - len(data))
            if new_bytes is None:
                data = None
            else:
                data = data + new_bytes

        return data


    def send_str(self, msg_type, data):
        """Sends a message over the network to the client.
        Params:
            msg_type - A unique integer identifier for the message.
            data     - A string containing the message data to send.

        This method will create the network packet necessary to send
        the message over the network.
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

            #Create message to send
            msg = chr(msg_type) + ''.join(map(chr, msg_len)) + data

            #Send message
            success = self._msg_sock.sendall(msg)
            if success != None:
                raise NetworkException('Could not send data on socket')


    def add_connect_listener(self, callback):
        """Adds a listener that is notified when a client connects.
        The callback will be passed the remote address that is received
        when socket.listen() returns.
        Params:
            callback - A listener that is notified on connect.
        """
        if callback not in self._conn_listeners:
            with self._listener_mutex:
                self._conn_listeners.append(callback)


    def add_data_listener(self, callback):
        """Adds a listener that is notified whenever data is received
        from the client.
        The callback will be passed a unique integer identifier of the
        message received and a list containing the message data.
        """
        if callback not in self._data_listeners:
            with self._listener_mutex:
                self._data_listeners.append(callback)


    def add_dc_listener(self, callback):
        """Adds a listener that is notified whenever a client disconnects.
        """
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
