# Utilities #

This directory contains useful scripts for dealing with the Raspberry Pi and
working with the OARK software.

Many of the scripts that were previously in this directory have been
superseded over time. Mike's build system eliminated the need for many of
them. The swap to Raspbian also made some of them defunct.


## arch/* ##

This directory contains scripts relevant to an Arch install of the OARK
software. It is unlikely that the project will ever go back to Arch, but if it
does then some of the groundwork has already been done. These scripts are not
updated to run the newest version, but they would be a good starting point for
a sudden change.


## gstreamer/ ##

This directory contains scripts that can be used to initiate gstreamer from
the command line. This has been superseded by the new python node for this
purpose, but these scripts have been kept mainly for documentation and
understanding purposes. It's much easier to understand gstreamer by running it
from the command line and the scripts in this directory contain examples of
how to build up the pipeline to do this.

Additionally, it is likely that gstreamer filters will have to be built at
some point in the future to support image processing. Performing the testing
for this will be much easier using the scripts contained here.


## config-ui.sh ##

Starts the config menu that appears on the TFT screen.


## oark ##

The init script for the oark software. This script should be copied (or
symlinked) to /etc/init.d and then update-rc.d needs to be run to install it.
It will call the ros-launch.sh file in the ros-pkg directory.

