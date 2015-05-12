package org.intelligentrobots.oarkcontroller;

import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.widget.TextView;

import org.intelligentrobots.oarkcontroller.streams.VideoStream;
import org.sipdroid.net.SipdroidSocket;

import java.io.IOException;
import java.net.SocketException;
import java.net.UnknownHostException;

/**
 * This is a surface callback
 */
public class VideoRenderer implements SurfaceHolder.Callback {
    private Thread mCodecThread;
    private SurfaceView mSurfaceView;

    public VideoRenderer(SurfaceView testSurfaceView) {
        mSurfaceView = testSurfaceView;
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {

        mCodecThread = new Thread() {
            public void run() {
                SipdroidSocket testSocket = null;
                try {
                    testSocket = new SipdroidSocket(5000);
                } catch (SocketException e) {
                    e.printStackTrace();
                } catch (UnknownHostException e) {
                    e.printStackTrace();
                }

                VideoStream testVideoStream = null;
                try {
                    testVideoStream = new VideoStream(testSocket, mSurfaceView);
                } catch (IOException e) {
                    e.printStackTrace();
                }

                testVideoStream.start();

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
