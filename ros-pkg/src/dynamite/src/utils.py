#! /usr/bin/env python
#
# Contains utility functions to capture
# small snippets of code that need to be
# run frequently.

from math import radians


# Converts ax12 coordinates to radians.
# AX12 coordinates are used to specify the angle
# of the horn on the servo. It is a single value that
# ranges from 0-1023.
# 0 is the most clockwise the horn can be rotated, and
# 1023 is the most counterclockwise the horn can be rotated.
#
# This coordinate value is converted to radians.
# 0 radians is the most clockwise the horn
# can be rotated.
# 5.236 radians is the most counterclockwise that the
# horn can be rotated
# (These values allow for a rotation of 300 degrees, the maximum
#  rotation that an AX12 allows when in position mode)
# Function truncates values outside of 0-1023 bound

# Note: This function depends on the init value set within the
# controller configuration file.

def ax12_to_rad(ax12_coord):
    if ax12_coord < 0: ax12_coord = 0
    if ax12_coord > 1023: ax12_coord = 1023
    
    return (ax12_coords / 1023.0 - 0.5) * radians(300)



#Takes a point in the interval (min1, max1) and converts it
# to a point in the interval (min2, max2).
# 
# After the function:
# (point - min1) / (point - max1) == (return_value - min2) / (return_value - max2)

def rescale(point, min1, max1, min2, max2):
    #validate
    if min1 > max1 - 1: min1 = max1 - 1
    if min2 > max2: min2 = max2
    if point < min1: point = min1
    if point > max1: point = max1

    #normalise point
    norm = float(point - min1) / float(max1 - min1)
    
    return min2 + norm * float(max2 - min2)

