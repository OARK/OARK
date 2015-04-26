package org.intelligentrobots.oarkcontroller.streams;

import android.media.MediaCodec;
import android.media.MediaFormat;
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

        MediaFormat format = MediaFormat.createVideoFormat("video/avc", 800, 600);
        m_codec.configure(format, m_surface, null, 0);

        m_codec.start();

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

        // TODO: Magic number, fix.
        byte[] buffer = new byte[BUFFER_SIZE + 12];
        m_rtpPacket = new RtpPacket(buffer, 0);

        m_running = true;

        while (m_running) {
            try {
                m_rtpSocket.receive(m_rtpPacket);

                int inputBufferIndex = m_codec.dequeueInputBuffer(SO_TIMEOUT);

                ByteBuffer inputBuffer;

                if (inputBufferIndex >= 0) {
                    inputBuffer = m_codec.getInputBuffer(inputBufferIndex);
                    inputBuffer.clear();

                    inputBuffer.put(m_rtpPacket.getPayload());

                    m_codec.queueInputBuffer(inputBufferIndex, 0, m_rtpPacket.getPayloadLength(), 0, 0);

                    inputBuffer.clear();
                }

                println("Current sequence number: " + m_rtpPacket.getSequenceNumber() + "\n");
                println("Payload type: " + m_rtpPacket.getPayloadType() + "\n");
                println("Payoad length: " + m_rtpPacket.getPayloadLength() + "\n");
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
