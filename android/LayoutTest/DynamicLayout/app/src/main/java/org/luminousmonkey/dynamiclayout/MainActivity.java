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

import org.luminousmonkey.dynamiclayout.ui.ControllerTable;
import org.yaml.snakeyaml.Yaml;

public class MainActivity extends Activity {
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        int numberOfSliders = 3;
        int numberOfSticks = 2;

        ControllerTable tbl = new ControllerTable(this);

        tbl.addSlider();
        tbl.addSlider();

        tbl.addAnalogStick();
        tbl.addAnalogStick();
        tbl.addAnalogStick();

        this.setContentView(tbl);
    }
}
