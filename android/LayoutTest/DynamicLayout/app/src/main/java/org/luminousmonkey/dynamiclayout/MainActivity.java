package org.luminousmonkey.dynamiclayout;

import android.app.Activity;
import android.os.Bundle;

import org.luminousmonkey.dynamiclayout.config.Config;
import org.luminousmonkey.dynamiclayout.config.Motor;
import org.luminousmonkey.dynamiclayout.ui.ControllerTable;

import java.io.IOException;
import java.util.Iterator;

public class MainActivity extends Activity {
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        int numberOfSliders = 3;
        int numberOfSticks = 2;

        try {
            Config testConfig = new Config(this);


        ControllerTable tbl = new ControllerTable(this);

        // Iterate through the list, anything that's absolute, slider.
        Iterator it = testConfig.getMotors().iterator();

        while (it.hasNext()) {
            Motor currentMotor = (Motor) it.next();

            if (currentMotor.usesAbsolutePosition()) {
                tbl.addSlider(currentMotor.getDisplayText(),
                              currentMotor.getMinPosition(),
                              currentMotor.getMaxPosition(),
                              currentMotor.getInitPosition());
            }
        }

        tbl.addSpacerRow();

        tbl.addAnalogStick();
        tbl.addAnalogStick();

        this.setContentView(tbl);

        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
