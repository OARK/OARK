#!/bin/bash
#
# Creates an access point with first argument as name
# Access point is bridged to eth0 for LAN access

CREATE_AP_HOME="/home/pi/create_ap"

if [ $# -eq 2 ]; then
		sudo ./create_ap wlan0 -m bridge eth0 $1
else
	echo "Usage: $0 <ap_name>"
fi

