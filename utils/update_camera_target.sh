#!/bin/bash

# Bash script to detect the IP of a device connected to the OARK robot
# software.

IFS=$'\n\t'

CAMERA_SCRIPT_LOCATION="/home/aldredmr/Projects/oark/utils/stream-video.sh"

while [ 1 ]
do
  # Get current IP
  CURRENT_IP=$(grep -oE '(\b[0-9]{1,3}\.){3}[0-9]{1,3}\b' < \
                    $CAMERA_SCRIPT_LOCATION)
  # Return the IP address of whatever is connected to the Robot port.
  TARGET_IP=$(netstat -ant | grep :1717 | grep ESTABLISHED | \
                 awk '{ print $5 }' | \
                 cut -d: -f1 | tail -n 1)

  echo "Current IP is $CURRENT_IP"
  echo "Target IP is $TARGET_IP"

  if [ "" != "$TARGET_IP" -a "0.0.0.0" != "$TARGET_IP" -a "$CURRENT_IP" != "$TARGET_IP" ]; then
    # Replace the IP Address in the camera script file.
    sed -i -r 's/(\b[0-9]{1,3}\.){3}[0-9]{1,3}\b'/"$TARGET_IP"/ \
        $CAMERA_SCRIPT_LOCATION

    sudo systemctl restart camera-stream
    else
      if [ "" != "$TARGET_IP" ]; then
          sudo systemctl stop camera-stream
      fi
  fi
  sleep 30
done
