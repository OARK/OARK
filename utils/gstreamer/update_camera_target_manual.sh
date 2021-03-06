#!/bin/bash

# Bash script to detect the IP of a device connected to the OARK robot
# software.

IFS=$'\n\t'

CAMERA_SCRIPT_LOCATION="/home/aldredmr/Projects/oark/utils/stream-video.sh"
DHCPD_CONF_LOCATION="/etc/dhcpd.conf"

if [ $# -ge 2 ]; then
    #TODO: tolower
    TARGET_MAC=$1
    HOST_NAME=$2

    # Get current IP
    CURRENT_IP=$(grep -oE '(\b[0-9]{1,3}\.){3}[0-9]{1,3}\b' < \
                    $CAMERA_SCRIPT_LOCATION)

    TARGET_IP=`grep -A 1 "$TARGET_MAC" "$DHCPD_CONF_LOCATION" | tail -n 1\
        | grep -oE '(\b[0-9]{1,3}\.){3}[0-9]{1,3}\b'`

    echo "Current IP is $CURRENT_IP"

    if [ "$TARGET_IP" "==" "0.0.0.0" ]; then
        #ERROR
        echo "Cannot target address 0.0.0.0 (all interfaces)"
        exit
    fi

    if [ "$TARGET_IP" "==" "" ]; then
        #Get larget IP in dhcpd file
        SORTED_EXISTING_IPS=`grep -E '^\s*fixed-address\s*(\b[0-9]{1,3}\.){3}[0-9]{1,3}\b;\s*$' "$DHCPD_CONF_LOCATION" | sed "s/^\s*fixed-address\s*\([^;]*\);\s*$/\1/" | sort`
        LAST_IP=`echo "$SORTED_EXISTING_IPS" | tail -n 1`

        if [ "$LAST_IP" "==" "" ]; then
            LAST_IP="192.168.12.13"
        fi

        TARGET_IP=`echo "$LAST_IP" | awk -F '.' '{printf("%d.%d.%d.%d", $1, $2, $3, $4 + 1)}'`

        echo "Creating new host in dhcpd.conf with IP: $TARGET_IP"

        echo "host $HOST_NAME {" >> "$DHCPD_CONF_LOCATION"
        echo "    hardware ethernet ${TARGET_MAC};" >> "$DHCPD_CONF_LOCATION"
        echo "    fixed-address ${TARGET_IP};" >> "$DHCPD_CONF_LOCATION"
        echo "}" >> "$DHCPD_CONF_LOCATION"
    fi
        
    # Replace the IP Address in the camera script file.
    sed -i -r 's/(\b[0-9]{1,3}\.){3}[0-9]{1,3}\b'/"$TARGET_IP"/ \
        $CAMERA_SCRIPT_LOCATION
else
    echo "Usage: $0 MAC_ADDR NAME"
    echo "MAC_ADDR is the MAC address of the device that you wish to have video streamed to. NAME is a *unique* name that this entry should have. NAME should not contain any spaces and should consist only of letters and underscores: e.g oark_controller (although oark_controller is taken). For a list of taken names, see /etc/dhcpd.conf. It should be of the form XX:XX:XX:XX:XX:XX where XX is a hexadecimal numeral with lowercase letters. Please reboot after a successful execution of this file for the changes to take effect."
fi
