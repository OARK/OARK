#!/bin/bash

# Slimline the Raspbian image, removing things we don't need.

set -euo pipefail
IFS=$'\n\t'

sudo apt-get purge xserver.* x11.* xarchiver xauth xkb-data console-setup \
     xinit lightdm lxde.* python-tk python3-tk scratch gtk.* libgtk.* openbox \
     libxt.* lxpanel gnome.* libqt.* libxcb.* libxfont.* lxmenu.* gvfs.* \
     xdg-.* desktop.* tcl.* shared-mime-info penguinspuzzle omxplayer gsfonts

sudo apt-get --yes autoremove
sudo apt-get upgrade
