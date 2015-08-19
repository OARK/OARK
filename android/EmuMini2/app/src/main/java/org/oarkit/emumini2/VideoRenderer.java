/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.emumini2;

import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

import org.sipdroid.net.RtpSocket;

import org.oarkit.emumini2.streams.VideoStream;
import org.sipdroid.net.SipdroidSocket;

import java.io.IOException;
import java.net.SocketException;
import java.net.UnknownHostException;

/**
 * This is expected to be added to the callback of the surface that is
 * intended for rendering of the video stream from the Robot.
 *
 * It will open a UDP port and listen for RtpH264 packets, rendering
 * the results to the Surface.
 */
public class VideoRenderer implements SurfaceHolder.Callback {
    /** Maximum blocking time, spent waiting for reading new bytes (ms) */
    public static final int SO_TIMEOUT = 1000;

    private SurfaceView mSurfaceView;
    private int mPortNumber;

    private SipdroidSocket mSocket;
    private RtpSocket mRtpSocket;

    // Thread that actually does everything.
    private Thread mStreamRenderer;

    private VideoRenderer() {};

    /**
     * After the video renderer is created here, it must be added as a
     * callback on the surface.
     *
     * e.g:
     *   testSurfaceView = (SurfaceView) findViewById(R.id.robotCameraView);
     *   testVideoRenderer = new VideoRenderer(testSurfaceView, 5000);
     *   testSurfaceView.getHolder().addCallback(testVideoRenderer);
     */
    public VideoRenderer(SurfaceView inSurfaceView, int inPortNumber) {
        mSurfaceView = inSurfaceView;
        mPortNumber = inPortNumber;
    }

    /**
     * Called on Surface creation.
     *
     * Starts a new thread that will read the video, and render onto
     * the given surface.
     */
    @Override
    public void surfaceCreated(SurfaceHolder holder) {
        try {
            mSocket = new SipdroidSocket(mPortNumber);
            mRtpSocket = new RtpSocket(mSocket);
            mRtpSocket.getDatagramSocket().setSoTimeout(SO_TIMEOUT);
        } catch (SocketException | UnknownHostException e) {
            e.printStackTrace();
        }

        try {
            if (mStreamRenderer == null) {
                mStreamRenderer = new VideoStream(mRtpSocket,
                                                  mSurfaceView);
            } else {
                Log.d("videoRenderer", "Stream Renderer is not null.");
            }
        } catch (IOException e) {
            e.printStackTrace();
        }

        mStreamRenderer.start();
    }

    public void stopRendering() {
        mStreamRenderer.interrupt();
        mStreamRenderer = null;

        mRtpSocket.close();
        mSocket.close();
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder,
                               int format,
                               int width,
                               int height) {

    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
        mStreamRenderer = null;
    }
}
