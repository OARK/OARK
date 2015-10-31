/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.emumini2.ui;

import java.util.ArrayList;

import android.content.Context;
import android.widget.Space;
import android.widget.TableLayout;
import android.widget.TableRow;

import android.widget.TextView;

public class ControllerTable extends TableLayout {

    private ArrayList<TableRow> tableRows = new ArrayList<TableRow>();
    private TableRow analogSticksRow;
    private boolean sticksAdded = false;

    public ControllerTable(Context context) {
        super(context);
        this.setOrientation(TableLayout.HORIZONTAL);

        TableLayout.LayoutParams layoutParams = new TableLayout.LayoutParams(
            TableRow.LayoutParams.MATCH_PARENT,
            TableRow.LayoutParams.MATCH_PARENT);

        layoutParams.setMargins(0, 0, 0, 0);
        this.setLayoutParams(layoutParams);

        analogSticksRow = new TableRow(this.getContext());
    }

    public ControllerSeekBar addSlider(String name, int min, int max, int init, int span) {
        TableRow newRow = new TableRow(this.getContext());
        ControllerSeekBar newControllerSeekBar =
            new ControllerSeekBar(this.getContext(), span);
        TextView newText = new TextView(this.getContext());

        newText.setText(name);
        newRow.addView(newText);
        newRow.addView(newControllerSeekBar);

        tableRows.add(newRow);

        TableRow.LayoutParams params = new TableRow.LayoutParams(
            TableRow.LayoutParams.MATCH_PARENT,
            TableRow.LayoutParams.WRAP_CONTENT);

        params.setMargins(0, 0, 0, 0);
        newRow.setLayoutParams(params);

        // Make sure to always add min to any value from the seekbar.
        newControllerSeekBar.setMax(min + max);
        newControllerSeekBar.setProgress(min + init);

        this.addView(newRow);

        return newControllerSeekBar;
    }

    public AnalogStick addAnalogStick() {

        if (!this.sticksAdded) {
            this.addView(analogSticksRow);
            this.sticksAdded = true;
        }

        AnalogStick newStick = new AnalogStick(this.getContext());

        TableRow.LayoutParams stickLayout = new TableRow.LayoutParams(
                TableRow.LayoutParams.MATCH_PARENT,
                TableRow.LayoutParams.WRAP_CONTENT, 1f);

        analogSticksRow.addView(newStick);
        newStick.setLayoutParams(stickLayout);

        return newStick;
    }

    public void addSpacerRow() {
        TableRow.LayoutParams params = new TableRow.LayoutParams(
            TableRow.LayoutParams.MATCH_PARENT,
            200, 1f);

        Space spacer = new Space(this.getContext());
        TableRow spacerRow = new TableRow(this.getContext());

        spacerRow.setLayoutParams(params);
        spacer.setLayoutParams(params);
        spacerRow.addView(spacer);

        this.addView(spacerRow);
    }
}
