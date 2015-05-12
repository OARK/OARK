package org.intelligentrobots.oarkcontroller.streams;

import android.media.MediaCodec;
import android.media.MediaCodecInfo;
import android.media.MediaCodecList;
import android.media.MediaFormat;
import android.media.MediaCodec.BufferInfo;
import android.util.Log;
import android.view.Surface;
import android.view.SurfaceView;

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
    public static final int BUFFER_SIZE = 4096;

    /** Maximum blocking time, spent waiting for reading new bytes (ms) */
    public static final int SO_TIMEOUT = 1000;

    private RtpSocket m_rtpSocket;
    private RtpPacket m_rtpPacket;

    private boolean m_running;

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

    public VideoStream(SipdroidSocket socket, SurfaceView outputSurfaceView) throws IOException {
        if (socket != null) {
            m_rtpSocket = new RtpSocket(socket);
        }

        m_surface = outputSurfaceView.getHolder().getSurface();
        m_codec = MediaCodec.createDecoderByType("video/avc");
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

    private MediaCodecInfo selectCodec(String mimeType) {
        int numCodecs = MediaCodecList.getCodecCount();
        for (int i = 0; i < numCodecs; i++) {
            MediaCodecInfo codecInfo = MediaCodecList.getCodecInfoAt(i);
            if (!codecInfo.isEncoder()) {
                continue;
            }
            String[] types = codecInfo.getSupportedTypes();
            for (int j = 0; j < types.length; j++) {
                if (types[j].equalsIgnoreCase(mimeType)) {
                    return codecInfo;
                }
            }
        }
        return null;
    }

    private int selectColorFormat(String mimeType) {
        MediaCodecInfo codecInfo = selectCodec(mimeType);
        int colorFormat = 0;
        MediaCodecInfo.CodecCapabilities capabilities =  codecInfo.getCapabilitiesForType(mimeType);
        for (int i = 0; i < capabilities.colorFormats.length; i++) {

            if (isRecognizedFormat(capabilities.colorFormats[i])) {
                colorFormat = capabilities.colorFormats[i];
                break;
            }
        }

        return colorFormat;
    }

    private boolean isRecognizedFormat(int colorFormat) {
        switch (colorFormat) {
            // these are the formats we know how to handle for this test
            case MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420Planar:
            case MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420PackedPlanar:
            case MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420SemiPlanar:
            case MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420PackedSemiPlanar:
            case MediaCodecInfo.CodecCapabilities.COLOR_TI_FormatYUV420PackedSemiPlanar:
                return true;
            default:
                return false;
        }
    }

    /**
     * Run the thread for handling the video stream.
     */
    public void run() {
        if (m_rtpSocket == null) {
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

                int inputBufferIndex = m_codec.dequeueInputBuffer(-1);
                ByteBuffer inputBuffer;

                if (inputBufferIndex >= 0) {
                    inputBuffer = m_codec.getInputBuffer(inputBufferIndex);

                    boolean bufferNotReady = true;
                    do {
                        m_rtpSocket.receive(m_rtpPacket);
                        Log.d(TAG, "RTP Payload Size: " + m_rtpPacket.getPayloadLength());

                        if (testRtpH264.doProcess(m_rtpPacket) == RtpH264.ProcessResult.BUFFER_PROCESSED_OK) {
                            bufferNotReady = !testRtpH264.ready();
                        }
                        Log.d(TAG, "Output Buffer Size: " + testRtpH264.currentLength());
                    } while (bufferNotReady);

                    Log.d(TAG, "Output from RTP depay is " + testRtpH264.getOutputBuffer().length + " bytes.");

                    byte[] transferArray = testRtpH264.getOutputBuffer();

                    inputBuffer.put(transferArray);

                    m_codec.queueInputBuffer(inputBufferIndex, 0, transferArray.length, 0, 0);
                    testRtpH264.clear();

                    int outIndex = m_codec.dequeueOutputBuffer(info, 10000);

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
                            Log.d("DocodeActivity", "Surface decoder given buffer " + outIndex +
                                " (size=" + info.size + ")");
                            m_codec.releaseOutputBuffer(outIndex, true);
                            break;
                    }
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
}
