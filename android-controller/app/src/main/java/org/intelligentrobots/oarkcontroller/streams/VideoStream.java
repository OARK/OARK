package org.intelligentrobots.oarkcontroller.streams;

import android.media.MediaCodec;
import android.media.MediaFormat;
import android.media.MediaCodec.BufferInfo;
import android.util.Log;
import android.view.Surface;
import android.view.SurfaceView;
import android.widget.TextView;

import org.sipdroid.net.RtpPacket;
import org.sipdroid.net.RtpSocket;
import org.sipdroid.net.SipdroidSocket;

import java.io.IOException;
import java.net.SocketException;
import java.nio.ByteBuffer;

/**
 * VideoStream is for receiving a video stream from the robot.
 */
public class VideoStream extends Thread {
    /** Size of the read buffer. */
    public static final int BUFFER_SIZE = 1024;

    /** Maximum blocking time, spent waiting for reading new bytes (ms) */
    public static final int SO_TIMEOUT = 1000;

    private RtpSocket m_rtpSocket;
    private RtpPacket m_rtpPacket;

    private boolean m_running;

    private TextView m_outputTextView;
    private Surface m_surface;

    private MediaCodec m_codec;
    private ByteBuffer[] m_decodeInputBuffers;

    private final static String TAG = "VideoStream: ";
    /**
     * RptStreamReceiver
     */
    public VideoStream(SipdroidSocket socket) {
        if (socket != null) {
            m_rtpSocket = new RtpSocket(socket);
        }
    }

    public VideoStream(SipdroidSocket socket, TextView outputTextView, SurfaceView outputSurfaceView) throws IOException {
        if (socket != null) {
            m_rtpSocket = new RtpSocket(socket);
        }

        m_surface = outputSurfaceView.getHolder().getSurface();
        m_codec = MediaCodec.createDecoderByType("video/avc");

        m_outputTextView = outputTextView;
    }

    /** Thread running? */
    public boolean isRunning() {
        return m_running;
    }

    /**
     * Empty the video stream.
     */
    void empty() {
        try {
            m_rtpSocket.getDatagramSocket().setSoTimeout(1);

            /*
             * This relies on throwing an exception when there's no more data
             * left.
             *
             * TODO: Fix it.
             */
            while(true) {
                m_rtpSocket.receive(m_rtpPacket);
            }
        } catch (SocketException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }

        try {
            m_rtpSocket.getDatagramSocket().setSoTimeout(SO_TIMEOUT);
        } catch (SocketException e) {
            e.printStackTrace();
        }
    }

    /**
     * Run the thread for handling the video stream.
     */
    public void run() {
        if (m_rtpSocket == null) {
            println("ERROR; RTP socket is null");
            return;
        }

        MediaFormat format = MediaFormat.createVideoFormat("video/avc", 640, 480);

        m_codec.configure(format, m_surface, null, 0);

        m_codec.start();

        // TODO: Magic number, fix.
        byte[] buffer = new byte[BUFFER_SIZE + 12];
        m_rtpPacket = new RtpPacket(buffer, 0);

        m_running = true;

        RtpH264 testRtpH264 = new RtpH264();
        BufferInfo info = new BufferInfo();

        while (m_running) {

            try {
                m_rtpSocket.receive(m_rtpPacket);

                int inputBufferIndex = m_codec.dequeueInputBuffer(SO_TIMEOUT);

                ByteBuffer inputBuffer;

                if (inputBufferIndex >= 0) {
                    Log.d(TAG, "BufferIndex: " + inputBufferIndex);
                    inputBuffer = m_codec.getInputBuffer(inputBufferIndex);
                    inputBuffer.clear();

                    testRtpH264.setPayload(m_rtpPacket.getPayload(), m_rtpPacket.getPayloadLength());
                    testRtpH264.parsePacket();

                    if (testRtpH264.isReady()) {
                        Log.d(TAG, "RtpH264 is ready");
                        inputBuffer.put(testRtpH264.getOutputBuffer());
                        m_codec.queueInputBuffer(inputBufferIndex, 0, testRtpH264.getOutputBuffer().length, 0, 0);
                        inputBuffer.clear();
                    }

                    int outIndex = m_codec.dequeueOutputBuffer(info, 10000);

                    Log.d(TAG, "OutIndex result: " + outIndex);

                    switch (outIndex) {
                        case MediaCodec.INFO_OUTPUT_BUFFERS_CHANGED:
                            Log.d("DecodeActivity", "INFO_OUTPUT_BUFFERS_CHANGED");
                            break;
                        case MediaCodec.INFO_OUTPUT_FORMAT_CHANGED:
                            Log.d("DecodeActivity", "New format " + m_codec.getOutputFormat());
                            break;
                        case MediaCodec.INFO_TRY_AGAIN_LATER:
                            Log.d("DecodeActivity", "dequeueOutputBuffer timed out!");
                            break;
                        default:
                            break;
                    }
                }




            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    /** Debug output */

    private void println(final String inStr) {
        if (m_outputTextView != null) {
            m_outputTextView.post(new Runnable() {
                @Override
                public void run() {
                    m_outputTextView.append(inStr);
                }
            });
        }
    }
}
