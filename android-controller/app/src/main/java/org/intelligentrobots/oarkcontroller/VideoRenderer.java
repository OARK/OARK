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
 * Created by aldredmr on 27/04/15.
 */
public class VideoRenderer implements SurfaceHolder.Callback {
    private Thread m_codecThread;
    private TextView m_testTextView;
    private SurfaceView m_testSurfaceView;

    public VideoRenderer(TextView testTextView, SurfaceView testSurfaceView) {
        m_testTextView = testTextView;
        m_testSurfaceView = testSurfaceView;
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {

        m_codecThread = new Thread() {
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
                    testVideoStream = new VideoStream(testSocket, m_testTextView, m_testSurfaceView);
                } catch (IOException e) {
                    e.printStackTrace();
                }

                testVideoStream.start();

            }
        };

        // When the surface is created, start the codec.
        m_codecThread.start();
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {

    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {

    }
}
