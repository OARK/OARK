package org.luminousmonkey.dynamiclayout;

import android.support.v7.app.ActionBarActivity;
import android.app.Activity;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;

import android.widget.Button;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.TextView;

import android.widget.SeekBar;
import android.widget.TableLayout;
import android.widget.TableRow;

import java.util.ArrayList;

public class MainActivity extends Activity {
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        int numberOfSliders = 3;
        int numberOfSticks = 2;

        ArrayList<TableRow> tableRows = new ArrayList<TableRow>();

        TableLayout tbl = new TableLayout(this);
        tbl.setOrientation(TableLayout.HORIZONTAL);

        for(int x = 0; x < numberOfSliders; x++) {
            TableRow currentRow = new TableRow(this);
            SeekBar currentSeekBar = new SeekBar(this);
            currentSeekBar.setMinimumWidth(150);

            currentRow.addView(currentSeekBar);
            tableRows.add(currentRow);

            TableRow.LayoutParams params = new TableRow.LayoutParams(
                    TableRow.LayoutParams.MATCH_PARENT,
                    TableRow.LayoutParams.MATCH_PARENT);
            params.span = numberOfSticks + 1;

            currentRow.setLayoutParams(params);
            tbl.addView(currentRow);
        }

        this.setContentView(tbl);
    }
}
