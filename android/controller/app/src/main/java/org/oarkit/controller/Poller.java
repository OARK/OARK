/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.controller;

import android.util.Log;

import org.oarkit.controller.messages.ControlMessage;
import org.oarkit.controller.ui.IRobotControl;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static java.lang.Thread.sleep;

/**
 * This class is for creating a thread that will poll the state of all
 * the controls and send the data at fixed time intervals.
 */
public class Poller {
    // Time in ms when updates should be sent to the robot.
    final private static long POLL_TIME = 200;

    private Transceiver mTransceiver;

    private Thread pollerThread;

    private volatile boolean mRunning;

    private List<IRobotControl> mControls;

    public Poller(Transceiver inTransceiver, List<IRobotControl> inControls) {
        mRunning = true;

        mControls = inControls;

        mTransceiver = inTransceiver;
        createAndStartThread();
    }

    private void createAndStartThread() {
        PollerThread ti = new PollerThread();
        pollerThread = new Thread(ti);
        pollerThread.start();
    }

    public void stop() {
        mRunning = false;
    }

    class PollerThread implements Runnable {
        public void run() {
            while (mRunning) {
                // Build up our list of values to send.
                List<Float> controlValues = new ArrayList<Float>();

                for (IRobotControl control : mControls) {
                    for (float value : control.getValues()) {
                        controlValues.add(value);
                    }

                }

                float[] messageContent = new float[controlValues.size()];

                int index = 0;
                for (float value : controlValues) {
                    messageContent[index++] = value;
                }
                ControlMessage message =
                    new ControlMessage(messageContent);

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
