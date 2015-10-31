/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.emumini2.ui;

import android.content.Context;
import android.widget.SeekBar;
import android.widget.TableRow;

public class ControllerSeekBar extends SeekBar implements IRobotControl {
    private final static int MAX_VALUE = 100;

    public ControllerSeekBar(final Context inContext, int inSpan) {
        super(inContext);

        TableRow.LayoutParams barLayout = new TableRow.LayoutParams(
            TableRow.LayoutParams.MATCH_PARENT,
            TableRow.LayoutParams.WRAP_CONTENT, 1f);

        barLayout.span = inSpan;
        this.setLayoutParams(barLayout);

        this.setMax(MAX_VALUE);
        this.setProgress(MAX_VALUE / 2);
    }

    public float[] getValues() {
        final int halfWay = (this.getMax() / 2);
        return new float[]{(float)((this.getProgress() - halfWay) / halfWay)};
    }
}
