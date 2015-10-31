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
import android.widget.SeekBar;

import org.oarkit.R;
import org.oarkit.emumini2.messages.InputResponse;
import org.oarkit.emumini2.networking.INetworkCallback;
import org.oarkit.emumini2.networking.InputsConfigTask;
import org.oarkit.emumini2.ui.ControllerTable;

import java.io.IOException;

public class ControlActivity extends Activity implements IInputRequestCallback {
    final private int DEFAULT_VIDEO_PORT = 5000;
    private final String targetIP = "10.0.1.192";

    private SurfaceView videoSurfaceView;
    private VideoRenderer videoRenderer;

    // Handles network communication.
    private Transceiver mTransceiver;

    // Polls the controls at a fixed interval and sends data to robot.
    private Poller controlPoller;

    // List of inputs from the robot.
    private List<Input> mInputList;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        // setContentView(R.layout.activity_control);

        /* Stop screen dimming */
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        /* Initialise video */
        // videoSurfaceView = (SurfaceView) findViewById(R.id.robotCameraView);
        FrameLayout mainFrame = new FrameLayout(this);
        SurfaceView videoSurfaceView = new SurfaceView(this);
        ControllerTable tbl = new ControllerTable(this);

        videoRenderer = new VideoRenderer(videoSurfaceView, DEFAULT_VIDEO_PORT);
        videoSurfaceView.getHolder().addCallback(videoRenderer);

        /* Initialise controls */
        // final AnalogStickView leftAnalog = (AnalogStickView) findViewById(R.id.left_analog_stick);
        // final AnalogStickView rightAnalog = (AnalogStickView) findViewById(R.id.right_analog_stick);
        // final SeekBar handSeek = (SeekBar) findViewById(R.id.handSeek);
        // final SeekBar wristSeek = (SeekBar) findViewById(R.id.wristSeek);
        // final SeekBar elbowSeek = (SeekBar) findViewById(R.id.elbowSeek);

        //tbl.addView(videoSurfaceView);

        mainFrame.addView(videoSurfaceView);
        mainFrame.addView(tbl);

        tbl.addSlider("Test", 0, 100, 0, 1);
        /* Connect to raspberry pi server */
        try {

            mTransceiver = new Transceiver(targetIP);
            mTransceiver.addCallback(new ReceiveTransceiverMessage());

            InputsConfigTask configTask = new InputsConfigTask();
            configTask.execute(mTransceiver);

            // Do InputRequest to robot and handling here.
            // controlPoller = new Poller(this, mTransceiver);
            this.setContentView(mainFrame);
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
        mInputList = inList.clone();
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
        // controlPoller.stop();
        videoRenderer.stopRendering();
        finish();
    }

    @Override
    public void onResume() {
        super.onResume();
    }
}
