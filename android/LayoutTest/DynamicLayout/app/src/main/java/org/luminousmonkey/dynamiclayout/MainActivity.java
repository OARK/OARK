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

        try {
            Config testConfig = new Config(this);

            ControllerTable tbl = new ControllerTable(this);

            // Iterate through the list, anything that's absolute, slider.
            Iterator it = testConfig.getMotors().iterator();

            // The following is horrible code, but it's a quick test.
            int stickCounter = 0;

            while (it.hasNext()) {
                Motor currentMotor = (Motor) it.next();

                stickCounter++;
            }

            it = testConfig.getMotors().iterator();
            while (it.hasNext()) {
                Motor currentMotor = (Motor) it.next();

                if (currentMotor.usesAbsolutePosition()) {
                    tbl.addSlider(currentMotor.getDisplayText(),
                                  currentMotor.getMinPosition(),
                                  currentMotor.getMaxPosition(),
                                  currentMotor.getInitPosition(),
                                  stickCounter);
                }
            }

            tbl.addSpacerRow();

            it = testConfig.getMotors().iterator();
            while (it.hasNext()) {
                Motor currentMotor = (Motor) it.next();

                if (!currentMotor.usesAbsolutePosition()) {
                    tbl.addAnalogStick();
                }
            }

            this.setContentView(tbl);

        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
