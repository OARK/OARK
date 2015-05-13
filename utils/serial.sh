#!/bin/bash
#
# Sets the baud rate of /dev/ttyAMA0 to 115200
#

PORT='/dev/ttyAMA0'

if [ -c $PORT ]; then
		stty -F $PORT 115200
		if [ $? -eq 0 ]; then
				echo "Done"
		else
				echo "Could not set $PORT to specified baud rate"
		fi
else
		echo "Port: $PORT does not exist"
fi


