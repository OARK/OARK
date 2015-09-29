package org.luminousmonkey.dynamiclayout;

import android.app.Activity;
import android.os.Bundle;

import org.luminousmonkey.dynamiclayout.config.Config;
import org.luminousmonkey.dynamiclayout.ui.ControllerTable;

import java.io.IOException;

public class MainActivity extends Activity {
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        int numberOfSliders = 3;
        int numberOfSticks = 2;

        try {
            Config testConfig = new Config(this);
        } catch (IOException e) {
            e.printStackTrace();
        }

        ControllerTable tbl = new ControllerTable(this);

        tbl.addSlider();
        tbl.addSlider();
        tbl.addSlider();

        tbl.addSpacerRow();

        tbl.addAnalogStick();
        tbl.addAnalogStick();

        this.setContentView(tbl);
    }
}
