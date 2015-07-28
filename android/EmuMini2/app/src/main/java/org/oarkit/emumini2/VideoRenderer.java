package org.oarkit.emumini2;

import android.view.SurfaceHolder;
import android.view.SurfaceView;

import org.oarkit.emumini2.streams.VideoStream;

import java.io.IOException;

/**
 * This is expected to be added to the callback of the surface that is
 * intended for rendering of the video stream from the Robot.
 *
 * It will open a UDP port and listen for RtpH264 packets, rendering
 * the results to the Surface.
 */
public class VideoRenderer implements SurfaceHolder.Callback {
    private Thread mCodecThread;
    private SurfaceView mSurfaceView;
    private int mPortNumber;

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

        mCodecThread = new Thread() {
            public void run() {

                VideoStream streamRenderer = null;
                try {
                    streamRenderer = new VideoStream(mPortNumber, mSurfaceView);
                } catch (IOException e) {
                    e.printStackTrace();
                }

                streamRenderer.start();
            }
        };

        // When the surface is created, start the codec.
        mCodecThread.start();
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {

    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {

    }
}
