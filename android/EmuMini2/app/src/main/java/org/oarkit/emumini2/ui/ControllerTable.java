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

    public void addSlider(ControllerSeekBar inControllerSeekBar) {
        TableRow newRow = new TableRow(this.getContext());
        TextView newText = new TextView(this.getContext());

        newText.setText(inControllerSeekBar.getTitle());
        newRow.addView(newText);
        newRow.addView(inControllerSeekBar);

        tableRows.add(newRow);

        TableRow.LayoutParams params = new TableRow.LayoutParams(
            TableRow.LayoutParams.MATCH_PARENT,
            TableRow.LayoutParams.WRAP_CONTENT);

        params.setMargins(0, 0, 0, 0);
        newRow.setLayoutParams(params);

        this.addView(newRow);
    }

    public void addStick(AnalogStick inStick) {

        if (!this.sticksAdded) {
            this.addView(analogSticksRow);
            this.sticksAdded = true;
        }

        TableRow.LayoutParams stickLayout = new TableRow.LayoutParams(
                TableRow.LayoutParams.MATCH_PARENT,
                TableRow.LayoutParams.WRAP_CONTENT, 1f);

        analogSticksRow.addView(inStick);
        inStick.setLayoutParams(stickLayout);
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
