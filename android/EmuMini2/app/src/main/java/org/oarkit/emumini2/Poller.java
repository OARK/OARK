/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.emumini2;

import android.app.Activity;
import android.util.Log;
import android.widget.SeekBar;

import org.oarkit.R;
import org.oarkit.emumini2.messages.ControlMessage;

import java.io.IOException;

import static java.lang.Thread.sleep;

/**
 * This class is for creating a thread that will poll the state of all
 * the controls and send the data at fixed time intervals.
 */
public class Poller {
    private String targetIP = "192.168.12.1";

    // Time in ms when updates should be sent to the robot.
    final private static long POLL_TIME = 200;

    private AnalogStickView leftAnalog;
    private AnalogStickView rightAnalog;
    private SeekBar handSeek;
    private SeekBar wristSeek;
    private SeekBar elbowSeek;

    private Transceiver mTransceiver;

    public Poller(Activity inActivity) {
        leftAnalog = (AnalogStickView) inActivity.findViewById(R.id.left_analog_stick);
        rightAnalog = (AnalogStickView) inActivity.findViewById(R.id.right_analog_stick);
        handSeek = (SeekBar) inActivity.findViewById(R.id.handSeek);
        wristSeek = (SeekBar) inActivity.findViewById(R.id.wristSeek);
        elbowSeek = (SeekBar) inActivity.findViewById(R.id.elbowSeek);

        // Try to connect to the robot.
        try {
            mTransceiver = new Transceiver(targetIP);
        } catch (IOException e) {
            Log.e("Poller", "Count not connect: " + e.getMessage());
        }

        createAndStartThread();
    }

    private void createAndStartThread() {
        PollerThread ti = new PollerThread();
        Thread pollerThread = new Thread(ti);
        pollerThread.start();
    }

    class PollerThread implements Runnable {
        public void run() {
            // TODO: Quick hack, fix this.
            try {
                sleep(3000);
            } catch (InterruptedException e) {

            }

            while (true) {
                // TODO: Poll a layout that is dynamically generated.
                float[] testingArray = new float[]{
                    (float) leftAnalog.getAnalogX(),
                    (float) leftAnalog.getAnalogY(),
                    (float) rightAnalog.getAnalogX(),
                    (float) rightAnalog.getAnalogY(),
                    (float) elbowSeek.getProgress() / elbowSeek.getMax(),
                    (float) wristSeek.getProgress() / wristSeek.getMax(),
                    (float) handSeek.getProgress() / handSeek.getMax()
                };

                ControlMessage message = new ControlMessage(testingArray);

                try {
                    mTransceiver.send(message);
                } catch (IOException e) {
                    Log.e("PollerThread", e.getMessage());
                }

                try {
                    sleep(POLL_TIME);
                } catch (InterruptedException e) {
                    Log.e("Poller", "Thread interrupted: " + e.getMessage());
                }
            }
        }
    }
}
