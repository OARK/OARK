package sep_17.emumini2;


import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;

import java.util.Timer;
import java.util.TimerTask;


/**
 * A splash full screen activity that displays the welcome screen for the application and then
 * creates a new activity.
 */
public class SplashActivity extends Activity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_splash);

        Timer startTimer = new Timer();
        startTimer.schedule(new TimerTask() {
            public void run() {
                Intent intent = new Intent(SplashActivity.this, IPActivity.class);
                startActivity(intent);
                finish();
            }
        }, 2000 );
    }
}