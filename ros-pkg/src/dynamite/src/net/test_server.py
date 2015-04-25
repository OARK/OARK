#! /usr/bin/env python
#
#
# This is a test server to allow testing of the 
# cmd_listener module. A test server needs to be
# run to allow a connection to occur.
#
# This test server takes five  parameters. 
# These are as follows:
# 
# 1     Period     - Number of times per second to send
#                    the parameter. 0 for send once.
#
# 2     Type       - The message type. An unsigned byte
# 3     Id         - The Id of the controller that the
#                    command is relevant to. 0 if not used.
# 4     Value      - The value of the message. A signed byte.
# 5     Duration   - The duration that the value should last
#                    for.
# 
# The first parameter (period) is used by this application to
# determine how often to send the parameter. The other 4 parameters
# are actually serialised and sent to the client.
# 


import sys
import signal
import socket
import time
import struct

PORT = 1717
ADDRESS = '127.0.0.1'


def cleanup_handler(signal, frame):
    print('Exit signal encountered. Program exiting...')
    try:
        if sock_fd is not None:
            sock_fd.close()
        if send_sock is not None:
            send_sock.close()
    except:
        pass


def send_data(sock_fd, type, id, value, duration):
    #Serialise data
    send_str = struct.pack('!BBBi', type, id, value, duration)
    #Add header
    send_str = str(len(send_str)) + send_str
    send_sock.sendall(send_str)


if __name__ == '__main__':
    # Allow clean exit on SIGINT
    signal.signal(signal.SIGINT, cleanup_handler)

    if len(sys.argv) < 6:
        print 'Usage: ./testserver.py '\
              '<period> <type> <id> <value> <duration>'
        sys.exit()

    period = int(sys.argv[1])
    type = int(sys.argv[2])
    id = int(sys.argv[3])
    value = int(sys.argv[4])
    duration = int(sys.argv[5])

    af_inet_addr = (ADDRESS, PORT)
    
    try:
        # Create socket
        sock_fd = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock_fd.bind(af_inet_addr)
        sock_fd.listen(1)

        (send_sock, dest_addr) = sock_fd.accept()
        print 'Client connected...'

        if period != 0:
            # Send data until error or SIGINT
            while True:
                sent_str = send_data(send_sock, type, id, value, duration)
                print 'Sending "%s" to client...'%sent_str
                time.sleep(period)
        elif period == 0:
                sent_str = send_data(send_sock, type, id, value, duration)
                print 'Sending "%s" to client...'%sent_str

        send_sock.close()
        sock_fd.close()
    except:
        print 'Error opening and connecting socket'
        raise
