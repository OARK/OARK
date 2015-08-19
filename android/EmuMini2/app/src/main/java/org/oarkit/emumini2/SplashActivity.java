/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.emumini2;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;

import org.oarkit.R;

import java.util.Timer;
import java.util.TimerTask;

/**
 * A splash full screen activity that displays the welcome screen for
 * the application and then creates a new activity.
 */
public class SplashActivity extends Activity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_splash);

        Timer startTimer = new Timer();
        startTimer.schedule(new TimerTask() {
            public void run() {
                Intent activityStarter = new Intent(getBaseContext(),
                                                    ControlActivity.class);
                startActivity(activityStarter);
                finish();
            }
        }, 2000 );
    }
}
