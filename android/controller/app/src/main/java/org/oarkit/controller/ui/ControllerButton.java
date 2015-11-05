/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.controller.ui;

import android.content.Context;
import android.view.View;
import android.widget.Button;

/**
 * Robot controller button.
 *
 * Because we're using a polling system, rather than an event based
 * system, button presses may get missed. With the other controllers,
 * this isn't much of an issue. However, because buttons are
 * momentary, we need to store if it's been pressed between polling
 * reads.
 */
public class ControllerButton extends Button implements IRobotControl {
    private final static float NOT_PRESSED = 0.0f;
    private final static float PRESSED = 1.0f;

    private boolean mPressed;

    public ControllerButton(final Context inContext, String inText) {
        super(inContext);
        mPressed = false;

        this.setOnClickListener(new View.OnClickListener() {
                public void onClick(View inView) {
                    mPressed = true;
                }});
    }

    public float[] getValues() {
        // IRobotControl expects a float array to be returned.
        float result = NOT_PRESSED;

        if (mPressed) {
            result = PRESSED;
            mPressed = false;
        }

        return new float[]{result};
    }
}
