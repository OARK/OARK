package org.luminousmonkey.dynamiclayout.ui;

import java.util.ArrayList;

import android.content.Context;
import android.view.ViewGroup;
import android.widget.SeekBar;
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

    public SeekBar addSlider() {
        TableRow newRow = new TableRow(this.getContext());
        SeekBar newSeekBar = new SeekBar(this.getContext());
        TextView newText = new TextView(this.getContext());

        newText.setText("Test");
        newRow.addView(newText);
        newRow.addView(newSeekBar);

        tableRows.add(newRow);

        TableRow.LayoutParams params = new TableRow.LayoutParams(
            TableRow.LayoutParams.MATCH_PARENT,
            TableRow.LayoutParams.WRAP_CONTENT);

        params.setMargins(0, 0, 0, 0);
        newRow.setLayoutParams(params);

        TableRow.LayoutParams barLayout = new TableRow.LayoutParams(
            TableRow.LayoutParams.MATCH_PARENT,
            TableRow.LayoutParams.WRAP_CONTENT, 1f);

        barLayout.span = 2;

        newSeekBar.setLayoutParams(barLayout);
        newSeekBar.setMax(100);

        this.addView(newRow);

        return newSeekBar;
    }

    public TextView addAnalogStick() {

        if (!this.sticksAdded) {
            this.addView(analogSticksRow);
            this.sticksAdded = true;
        }

        TextView newStick = new TextView(this.getContext());
        newStick.setText("Test");

        TableRow.LayoutParams stickLayout = new TableRow.LayoutParams(
                TableRow.LayoutParams.MATCH_PARENT,
                TableRow.LayoutParams.WRAP_CONTENT, 1f);

        analogSticksRow.addView(newStick);
        newStick.setLayoutParams(stickLayout);

        return newStick;
    }
}
