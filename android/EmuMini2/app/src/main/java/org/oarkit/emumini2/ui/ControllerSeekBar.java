/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.emumini2.ui;

import android.content.Context;
import android.widget.SeekBar;
import android.widget.TableRow;

/**
 * Robot controller seekbar.
 *
 * This class creates a slider type of control, one that is used for
 * absolute positioning. The value it returns will be between -1.0 and
 * 1.0, with 0.0 being the middle of the slider.
 */
public class ControllerSeekBar extends SeekBar implements IRobotControl {
    private final static int MAX_VALUE = 100;
    private String mTitle;

    public ControllerSeekBar(final Context inContext, String inTitle) {
        super(inContext);

        this.setMax(MAX_VALUE);
        this.setProgress(MAX_VALUE / 2);
    }

    public void setTitle(String inTitle) {
        mTitle = inTitle;
    }

    public void setSpan(int inSpan) {
        TableRow.LayoutParams barLayout = new TableRow.LayoutParams(
            TableRow.LayoutParams.MATCH_PARENT,
            TableRow.LayoutParams.WRAP_CONTENT, 1f);

        barLayout.span = inSpan;
        this.setLayoutParams(barLayout);
    }

    public String getTitle() {
        return mTitle;
    }

    public float[] getValues() {
        final float halfWay = (this.getMax() / 2);
        return new float[]{(this.getProgress() - halfWay) / halfWay};
    }
}
