package sep_17.intelligentrobots.oarkcontroller.streams;

import android.media.MediaCodec;
import android.media.MediaFormat;
import android.media.MediaCodec.BufferInfo;
import android.util.Log;
import android.view.Surface;
import android.view.SurfaceView;

import sep_17.sipdroid.net.RtpPacket;
import sep_17.sipdroid.net.RtpSocket;
import sep_17.sipdroid.net.SipdroidSocket;

import java.io.IOException;
import java.io.InterruptedIOException;
import java.net.SocketException;
import java.nio.ByteBuffer;

/**
 * VideoStream is for receiving a video stream from the robot.
 */
public class VideoStream extends Thread {
    /**
     * Size of input buffer from network.
     * TODO: Update to find MTU of network device used, and use that.
     */
    public static final int BUFFER_SIZE = 2048;

    /** Maximum blocking time, spent waiting for reading new bytes (ms) */
    public static final int SO_TIMEOUT = 1000;

    public static final int DEFAULT_WIDTH = 640;
    public static final int DEFAULT_HEIGHT = 480;

    private RtpSocket mRtpSocket;
    private RtpPacket mRtpPacket;

    private boolean mRunning;

    private Surface mSurface;

    private MediaCodec mCodec;

    private final static String TAG = "VideoStream: ";

    /**
     * VideoStream
     *
     * Doesn't make sense to build an instance without socket and
     * output surface.
     */
    private VideoStream() {};

    public VideoStream(SipdroidSocket socket, SurfaceView outputSurfaceView) throws IOException {
        if (socket != null) {
            mRtpSocket = new RtpSocket(socket);
        }

        mSurface = outputSurfaceView.getHolder().getSurface();
        mCodec = MediaCodec.createDecoderByType("video/avc");
    }

    /** Thread running? */
    public boolean isRunning() {
        return mRunning;
    }

    /**
     * Empty the video stream.
     */
    void empty() {
        try {
            mRtpSocket.getDatagramSocket().setSoTimeout(1);

            /*
             * This relies on throwing an exception when there's no more data
             * left.
             *
             * TODO: Fix it.
             */
            while(true) {
                mRtpSocket.receive(mRtpPacket);
            }
        } catch (SocketException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }

        try {
            mRtpSocket.getDatagramSocket().setSoTimeout(SO_TIMEOUT);
        } catch (SocketException e) {
            e.printStackTrace();
        }
    }

    /**
     * Run the thread for handling the video stream.
     */
    public void run() {
        Log.d(TAG, "Running");
        if (mRtpSocket == null) {
            Log.d(TAG, "Rtp Socket is null");
            return;
        }

        try {
            mRtpSocket.getDatagramSocket().setSoTimeout(SO_TIMEOUT);
        } catch (SocketException e) {
            e.printStackTrace();
        }

        Log.d(TAG, "Setting format");

        MediaFormat format = MediaFormat.createVideoFormat("video/avc",
                                                           DEFAULT_WIDTH,
                                                           DEFAULT_HEIGHT);

        RtpH264 rtpH264Depacket = new RtpH264();

        byte[] buffer = new byte[BUFFER_SIZE];
        mRtpPacket = new RtpPacket(buffer, 0);

        do {
            try {
                mRtpSocket.receive(mRtpPacket);
            } catch (InterruptedIOException ex) {
                Log.d(TAG, "Interrupted IO Exception: " + ex.toString());
            } catch (IOException ex) {
                // If we get an exception, just clear out the MediaCodec buffers.
                Log.d(TAG, "IO Exception" + ex.toString());
                rtpH264Depacket.discardBuffer();
            }


            rtpH264Depacket.doProcess(mRtpPacket);

        } while (!rtpH264Depacket.isConfigReady());

        format.setByteBuffer("csd-0", ByteBuffer.wrap(rtpH264Depacket.getSps()));
        format.setByteBuffer("csd-1", ByteBuffer.wrap(rtpH264Depacket.getPps()));

        mCodec.configure(format, mSurface, null, 0);
        mCodec.start();

        mRunning = true;

        rtpH264Depacket = new RtpH264();
        BufferInfo info = new BufferInfo();

        ByteBuffer[] inputBuffers = mCodec.getInputBuffers();

        while (mRunning) {
                int inputBufferIndex = mCodec.dequeueInputBuffer(-1);
                ByteBuffer inputBuffer;

                if (inputBufferIndex >= 0) {
                    inputBuffer = inputBuffers[inputBufferIndex];

                    boolean bufferNotReady = true;
                    do {
                        try {
                            mRtpSocket.receive(mRtpPacket);
                        } catch (InterruptedIOException ex) {
                            Log.d(TAG, "Interrupted IO Exception: " + ex.toString());
                        } catch (IOException ex) {
                            // If we get an exception, just clear out the MediaCodec buffers.
                            Log.d(TAG, "IO Exception" + ex.toString());
                            rtpH264Depacket.discardBuffer();
                        }

                        if (rtpH264Depacket.doProcess(mRtpPacket) ==
                            RtpH264.ProcessResult.BUFFER_PROCESSED_OK) {
                            bufferNotReady = !rtpH264Depacket.ready() &&
                                !rtpH264Depacket.isConfigPacket();
                        }

                    } while (bufferNotReady);

                    byte[] transferArray = rtpH264Depacket.getOutputBuffer();

                    inputBuffer.clear();
                    inputBuffer.put(transferArray);

                    mCodec.queueInputBuffer(inputBufferIndex, 0, transferArray.length, 0, 0);
                    rtpH264Depacket.clear();

                    int outIndex = mCodec.dequeueOutputBuffer(info, 10000);

                    switch (outIndex) {
                    case MediaCodec.INFO_OUTPUT_BUFFERS_CHANGED:
                        break;
                    case MediaCodec.INFO_OUTPUT_FORMAT_CHANGED:
                        Log.d("DecodeActivity", "New format " + mCodec.getOutputFormat());
                        break;
                    case MediaCodec.INFO_TRY_AGAIN_LATER:
                        // Log.d("DecodeActivity", "dequeueOutputBuffer timed out!");
                        // This is not an error, just means decoder isn't ready.
                        break;
                    default:
                        mCodec.releaseOutputBuffer(outIndex, (info.size != 0));
                        break;
                    }
                }
        }
    }
}