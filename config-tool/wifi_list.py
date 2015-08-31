#!/usr/bin/env python
#
# Quick test of getting the output from the iwlist command.

import os
import re
import subprocess

def get_channel_list(interface):
    try:
        wifi_channels = subprocess.check_output(['iwlist', interface, 'channel'], stderr=open(os.devnull, 'wb')).splitlines()
    except OSError as e:
        print "Unable to run 'iwlist' command, possibly not present?\n"
        raise

    CHANNEL_LINE = re.compile('Channel (\d+) : (\d+\.\d+) GHz')
    channel_list = {}
    for line in wifi_channels:
        stripped_line = line.strip()
        channel = CHANNEL_LINE.match(stripped_line)
        if channel != None:
            channel_list[int(channel.group(1))] = float(channel.group(2))
    return channel_list

if __name__ == "__main__":
    print get_channel_list('wlan0')
