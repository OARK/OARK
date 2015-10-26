/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.emumini2;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.SurfaceView;
import android.view.WindowManager;
import android.widget.SeekBar;

import org.oarkit.R;

public class ControlActivity extends Activity {
    final private int DEFAULT_VIDEO_PORT = 5000;

    private SurfaceView videoSurfaceView;
    private VideoRenderer videoRenderer;

    // Polls the controls at a fixed interval and sends data to robot.
    private Poller controlPoller;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_control);

        /* Stop screen dimming */
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        /* Initialise video */
        videoSurfaceView = (SurfaceView) findViewById(R.id.robotCameraView);
        videoRenderer = new VideoRenderer(videoSurfaceView, DEFAULT_VIDEO_PORT);
        videoSurfaceView.getHolder().addCallback(videoRenderer);

        /* Initialise controls */
        final AnalogStickView leftAnalog = (AnalogStickView) findViewById(R.id.left_analog_stick);
        final AnalogStickView rightAnalog = (AnalogStickView) findViewById(R.id.right_analog_stick);
        final SeekBar handSeek = (SeekBar) findViewById(R.id.handSeek);
        final SeekBar wristSeek = (SeekBar) findViewById(R.id.wristSeek);
        final SeekBar elbowSeek = (SeekBar) findViewById(R.id.elbowSeek);

        /* Connect to raspberry pi server */
        try {
            controlPoller = new Poller(this);
        }
        catch(Exception e) {
            finish();
        }
    }

    @Override
    public void onPause() {
        super.onPause();
        videoRenderer.stopRendering();
        finish();
    }

    @Override
    public void onResume() {
        super.onResume();
    }
}
