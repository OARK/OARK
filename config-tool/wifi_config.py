#!/usr/bin/env python

import locale
import sys
from dialog import Dialog

import wifi_list

# Set the locale
locale.setlocale(locale.LC_ALL, '')

# Our dialog
d = Dialog(dialog="dialog")
d.add_persistent_args(["--backtitle", "Wifi Config"])

# Defaults
DEFAULT_SSID="robot"
DEFAULT_PASSWORD="oark"
DEFAULT_COUNTRY_CODE="AU"
DEFAULT_CHANNEL=0

DEFAULT_TEXTBOX_HEIGHT=10
DEFAULT_TEXTBOX_WIDTH=30

def main_menu(dialog):
    "Display the main menu."

    do_not_exit = True

    while(do_not_exit):
        code, tag = dialog.menu("What do you want to edit?:",
                                choices=[("SSID", "Change SSID"),
                                         ("Password", "Change Password"),
                                         ("Country Code", "Change country code"),
                                         ("Channel", "Change channel")])
        if code == dialog.DIALOG_OK:
        # tag is menu selected.
            if tag == "SSID":
                change_ssid(dialog, DEFAULT_SSID)
            elif tag == "Password":
                change_password(dialog, DEFAULT_PASSWORD)
            elif tag == "Country Code":
                change_country_code(dialog, DEFAULT_COUNTRY_CODE)
            elif tag == "Channel":
                change_channel(dialog, DEFAULT_CHANNEL)
        else:
            do_not_exit = False

def change_ssid(dialog, current_ssid):
    result = current_ssid

    code, ssid = dialog.inputbox("SSID", DEFAULT_TEXTBOX_HEIGHT,
                                 DEFAULT_TEXTBOX_WIDTH, current_ssid)

    if code == dialog.DIALOG_OK:
        result = ssid

    return result

def change_password(dialog, current_password):
    result = current_password

    code, password = dialog.inputbox("Password", DEFAULT_TEXTBOX_HEIGHT,
                                     DEFAULT_TEXTBOX_WIDTH, current_password)

    if code == dialog.DIALOG_OK:
        result = password

    return result

def change_country_code(dialog, current_country_code):
    result = current_country_code

    code, country_code = dialog.radiolist("Country Code",
                                          choices=[("AU", "Australia", "on"),
                                                   ("CN", "China", "off"),
                                                   ("US", "United States", "off")])
    if code == dialog.DIALOG_OK:
        result = country_code

    return result

def list_channels(current_channel):
    channels = wifi_list.get_channel_list('wlan0')
    dialog_list = []

    if channels:
        auto_scan = "off"
        if current_channel == 0:
            auto_scan = "on"

        dialog_list.append(tuple(["0","Auto Scan", auto_scan]))

        for channel_no in channels:
            selected = "off"

            if channel_no == current_channel:
                selected = "on"

            channel_string = "Channel {0}, Freq {1} GHz".format(channel_no, channels[channel_no])
            dialog_list.append(tuple([str(channel_no), channel_string, selected]))

    return dialog_list


def change_channel(dialog, current_channel):
    result = current_channel

    channel_choices = list_channels(current_channel)
    code, channel = dialog.radiolist("Channel",
                                     choices=channel_choices)

    if code == dialog.DIALOG_OK:
        result = int(channel)

    return result

main_menu(d)
