#!/usr/bin/env python

# Copyright (c) 2015 Open Academic Robot Kit

from configobj import ConfigObj

import fileinput

DEFAULT_CONFIG_FILE = '/etc/hostapd/hostapd.conf'

class WifiConfig:
    def __init__(self, filename=DEFAULT_CONFIG_FILE):
        self.filename = filename
        self.config = ConfigObj(self.filename)

    def get_interface(self):
        return self.config['interface']

    def get_ssid(self):
        return self.config['ssid']

    def get_hwmode(self):
        return self.config['hw_mode']

    def get_channel(self):
        return int(self.config['channel'])

    def get_wpa_passphrase(self):
        return self.config['wpa_passphrase']

    def get_country_code(self):
        return self.config['country_code']

    def set_ssid(self, ssid):
        self.config['ssid'] = ssid

    def set_hwmode(self, hwmode):
        self.config['hw_mode'] = hwmode

    def set_channel(self, channel):
        self.config['channel'] = int(channel)

    def set_wpa_passphrase(self, wpa_passphrase):
        self.config['wpa_passphrase'] = wpa_passphrase

    def set_country_code(self, country_code):
        self.config['country_code'] = country_code

    def save(self):
        self.config.write()
        for line in fileinput.input(self.filename, inplace=True):
            print(line.replace(' = ', '=')),
