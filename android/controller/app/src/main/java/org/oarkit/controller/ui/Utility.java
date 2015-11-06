/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.controller.ui;

import android.graphics.Color;

/**
 * Utility functions for the UI.
 */
public class Utility {
    final static double calculateLuminance(int inColor) {
        final double RED_COMPONENT = 0.2126;
        final double GREEN_COMPONENT = 0.7152;
        final double BLUE_COMPONENT = 0.0722;
        final double COMPONENT_MAX = 256.0;

        double result = 1.0;

        if (android.R.color.transparent != inColor) {
            final double red = Color.red(inColor) / COMPONENT_MAX;
            final double green = Color.green(inColor) / COMPONENT_MAX;
            final double blue = Color.blue(inColor) / COMPONENT_MAX;

            result = ((RED_COMPONENT * red) +
                      (GREEN_COMPONENT * green) +
                      (BLUE_COMPONENT * blue) / COMPONENT_MAX);
        }

        return result;
    }
}
