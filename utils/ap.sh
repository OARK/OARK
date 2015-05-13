#!/bin/bash
#
# Creates an access point with first argument as name

CREATE_AP_HOME="/home/pi/create_ap"

if [ $# -eq 1 ]; then
		sudo ${CREATE_AP_HOME}/create_ap wlan0 -n $1
else
	echo "Usage: $0 <ap_name>"
fi

