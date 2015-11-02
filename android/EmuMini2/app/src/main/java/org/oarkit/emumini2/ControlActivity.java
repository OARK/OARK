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
import android.widget.FrameLayout;

import org.oarkit.emumini2.messages.InputResponse;
import org.oarkit.emumini2.messages.rosmessages.Input;
import org.oarkit.emumini2.networking.IInputRequestCallback;
import org.oarkit.emumini2.networking.INetworkCallback;
import org.oarkit.emumini2.networking.InputsConfigTask;
import org.oarkit.emumini2.ui.ControllerMapping;
import org.oarkit.emumini2.ui.ControllerTable;
import org.oarkit.emumini2.ui.IRobotControl;

import java.io.IOException;
import java.util.List;

public class ControlActivity extends Activity implements IInputRequestCallback {
    final private int DEFAULT_VIDEO_PORT = 5000;
    private final String targetIP = "192.168.6.22";

    private VideoRenderer videoRenderer;
    private FrameLayout mainFrame;

    // Handles network communication.
    private Transceiver mTransceiver;

    // Polls the controls at a fixed interval and sends data to robot.
    private Poller controlPoller;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        // setContentView(R.layout.activity_control);

        /* Stop screen dimming */
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        mainFrame = new FrameLayout(this);
        SurfaceView videoSurfaceView = new SurfaceView(this);

        videoRenderer = new VideoRenderer(videoSurfaceView, DEFAULT_VIDEO_PORT);
        videoSurfaceView.getHolder().addCallback(videoRenderer);

        mainFrame.addView(videoSurfaceView);

        /* Connect to raspberry pi server */
        try {

            mTransceiver = new Transceiver(targetIP);
            mTransceiver.addCallback(new ReceiveTransceiverMessage());

            InputsConfigTask configTask = new InputsConfigTask();
            configTask.setCallback(this);
            configTask.execute(mTransceiver);
        }
        catch(Exception e) {
            Log.e("ControlActivity", "Caught exception: " + e.getMessage());
            Log.e("controlActivity", "Exiting application.");
            finish();
        }
    }

    /**
     * Called when when the robot information for the available inputs
     * has been received.
     */
    public void updateInputControllers(List<Input> inList) {
        // Keep a copy of the list so we know the order values are to
        // be sent to the robot.
        Log.i("ControlActivity", "Received inputs");

        ControllerTable tbl = new ControllerTable(this);
        List<IRobotControl> controls =
                ControllerMapping.mapAndCreateControllers(this, tbl, inList);
        mainFrame.addView(tbl);

        this.setContentView(mainFrame);

        for (Input inp : inList) {
            Log.i("ControlActivity", "Name: " + inp.getName() + "Type: " + inp.getType());
        }

        // Start polling the controls now.
        controlPoller = new Poller(mTransceiver, controls);
    }

    class ReceiveTransceiverMessage implements INetworkCallback {
        public void handleMessage(byte[] inMessage) {
            switch (inMessage[0]) {
            case InputResponse.OARK_TYPE:
                //     handleInputResponse(inMessage);
                Log.e("ControlActivity", "InputResponse not implemented.");
            default:
                Log.i("ControlActivity", "Unhandled message type: " +
                      inMessage[0]);
            }
        }
    }

    @Override
    public void onPause() {
        super.onPause();
        try {
            mTransceiver.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        controlPoller.stop();
        videoRenderer.stopRendering();
        finish();
    }

    @Override
    public void onResume() {
        super.onResume();
    }
}
