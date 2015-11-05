#!/bin/sh
#
# Start the config UI on boot.
#
# Adapted from raspi-config.sh.
# https://github.com/asb/raspi-config/blob/master/sample_profile_d.sh

if [ $(id -u) -ne 0 ]; then
  printf "\nRun /opt/oark/config-tool/wifi_config.py to setup Wifi.\n\n"
else
  /opt/oark/config-tool/wifi_config.py
  exec login -f pi
fi
