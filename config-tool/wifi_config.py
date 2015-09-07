#!/usr/bin/env python

import locale
import sys
from dialog import Dialog

import wifi_config_file
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

HW_MODE_G_CUTOFF_CHANNEL=14

COUNTRIES = {'AU': 'Australia',
             'CN': 'China',
             'US': 'United States'}

def main_menu(dialog):
    "Display the main menu."

    do_not_exit = True

    wifi_config = wifi_config_file.WifiConfig()

    while(do_not_exit):
        code, tag = dialog.menu("What do you want to edit?:",
                                choices=[("SSID", "Change SSID"),
                                         ("Password", "Change Password"),
                                         ("Country Code", "Change country code"),
                                         ("Channel", "Change channel"),
                                         ("Save", "Save Config")])
        if code == dialog.DIALOG_OK:
        # tag is menu selected.
            if tag == "SSID":
                wifi_config.set_ssid(change_ssid(dialog, wifi_config.get_ssid()))
            elif tag == "Password":
                wifi_config.set_wpa_passphrase(change_password(dialog, wifi_config.get_wpa_passphrase()))
            elif tag == "Country Code":
                wifi_config.set_country_code(change_country_code(dialog, wifi_config.get_country_code()))
            elif tag == "Channel":
                wifi_config.set_channel(change_channel(dialog, wifi_config.get_channel()))
            elif tag == "Save":
                set_hw_mode(wifi_config.get_channel(), wifi_config)
                wifi_config.save()
                dialog.msgbox('Config file saved')
        else:
            do_not_exit = False

def set_hw_mode(channel, wifi_config):
    if channel <= HW_MODE_G_CUTOFF_CHANNEL:
        wifi_config.set_hwmode('g')
    else:
        wifi_config.set_hwmode('a')

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
    country_choices = list_country_codes(current_country_code)

    code, country_code = dialog.radiolist("Country Code",
                                          choices=country_choices)
    if code == dialog.DIALOG_OK:
        result = country_code

    return result

def list_country_codes(current_country_code):
    country_list = []

    for country in COUNTRIES:
        selected = 'off'

        if country == current_country_code:
            selected = 'on'

        country_list.append(tuple([country, COUNTRIES[country], selected]))

    return country_list

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
