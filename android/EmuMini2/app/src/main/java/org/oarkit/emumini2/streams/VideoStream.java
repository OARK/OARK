package org.oarkit.emumini2.streams;

import android.media.MediaCodec;
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
import java.net.SocketTimeoutException;
import java.net.UnknownHostException;
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

    /** Max time allowed to decode a frame. */
    public static final int DECODE_TIMEOUT = 1000;

    public static final int DEFAULT_WIDTH = 640;
    public static final int DEFAULT_HEIGHT = 480;

    private int mPortNumber;
    private SipdroidSocket mSocket;
    private RtpSocket mRtpSocket;
    private RtpPacket mRtpPacket;

    private boolean mRunning;

    /** Surface video frames are rendered onto. */
    private Surface mSurface;

    /** Video decoder. */
    private MediaCodec mCodec;

    /**
     * VideoStream
     *
     * Doesn't make sense to build an instance without socket and
     * output surface.
     */
    private VideoStream() {};

    public VideoStream(int inPortNumber, SurfaceView outputSurfaceView) throws IOException {

        mSocket = null;
        mPortNumber = inPortNumber;

        try {
            mSocket = new SipdroidSocket(mPortNumber);
        } catch (SocketException | UnknownHostException e) {
            e.printStackTrace();
        }

        mRtpSocket = new RtpSocket(mSocket);

        mSurface = outputSurfaceView.getHolder().getSurface();
        mCodec = MediaCodec.createDecoderByType("video/avc");
    }

    /*
     * Open a new socket for receiving Rtp packets. It's expected that
     * mPortNumber is correctly set.
     */
    private void openRtpSocket() {
        mRtpSocket.close();
        mSocket.close();

        try {
            mSocket = new SipdroidSocket(mPortNumber);
            mRtpSocket = new RtpSocket(mSocket);
            mRtpSocket.getDatagramSocket().setSoTimeout(SO_TIMEOUT);
        } catch (SocketException | UnknownHostException e) {
            e.printStackTrace();
        }
    }

    /**
     * Run the thread for handling the video stream.
     */
    public void run() {
        try {
            mRtpSocket.getDatagramSocket().setSoTimeout(SO_TIMEOUT);
        } catch (SocketException e) {
            e.printStackTrace();
        }

        RtpH264 rtpH264Depacket = new RtpH264();

        mRtpPacket = new RtpPacket(new byte[BUFFER_SIZE], 0);

        MediaFormat format = waitForConfigPackets(mRtpSocket,
                                                  mRtpPacket,
                                                  rtpH264Depacket);

        mCodec.configure(format, mSurface, null, 0);
        mCodec.start();

        mRunning = true;

        rtpH264Depacket = new RtpH264();
        BufferInfo info = new BufferInfo();

        /** Buffers to push data into for the codec to decode and display. */
        ByteBuffer[] inputBuffers = mCodec.getInputBuffers();

        while (mRunning) {
                int inputBufferIndex = mCodec.dequeueInputBuffer(-1);
                ByteBuffer inputBuffer;

                if (inputBufferIndex >= 0) {
                    // Input buffer available
                    inputBuffer = inputBuffers[inputBufferIndex];

                    boolean bufferNotReady = true;
                    do {
                        receiveRtpPacket(mRtpSocket, mRtpPacket,
                                         rtpH264Depacket);

                        if (rtpH264Depacket.doProcess(mRtpPacket) ==
                            RtpH264.ProcessResult.BUFFER_PROCESSED_OK) {

                            // Skip
                            bufferNotReady = !rtpH264Depacket.ready() &&
                                !rtpH264Depacket.isConfigPacket();
                        }

                    } while (bufferNotReady);

                    byte[] transferArray = rtpH264Depacket.getOutputBuffer();

                    inputBuffer.clear();
                    inputBuffer.put(transferArray);

                    mCodec.queueInputBuffer(inputBufferIndex, 0, transferArray.length, 0, 0);
                    rtpH264Depacket.clear();

                    int outIndex = mCodec.dequeueOutputBuffer(info, DECODE_TIMEOUT);

                    switch (outIndex) {
                    case MediaCodec.INFO_OUTPUT_BUFFERS_CHANGED:
                        // Depreciated, but still a result, so this is
                        // here to catch the case so default doesn't
                        // have to check for this result.
                        break;
                    case MediaCodec.INFO_OUTPUT_FORMAT_CHANGED:
                        Log.d("DecodeActivity", "New format " + mCodec.getOutputFormat());
                        break;
                    case MediaCodec.INFO_TRY_AGAIN_LATER:
                        // This is not an error, just means decoder isn't ready.
                        break;
                    default:
                        // Something in the buffer, render it to the surface.
                        mCodec.releaseOutputBuffer(outIndex, (info.size != 0));
                        break;
                    }
                }
        }
    }

    // Receive RTP packet.
    private void receiveRtpPacket(RtpSocket rtpSocket,
                                  RtpPacket rtpPacket,
                                  RtpH264 rtpH264) {
        try {
            rtpSocket.receive(rtpPacket);
        } catch (SocketTimeoutException ex) {
            // It seems that when we have a timeout we have no choice
            // but to close and open a new socket.
            openRtpSocket();
        } catch (IOException ex) {
            // If we get an exception, just clear out the MediaCodec buffers.
            rtpH264.discardBuffer();
        }
    }

    // Ensure that the decoder has the config packets before trying to
    // show the video stream.
    private MediaFormat waitForConfigPackets(RtpSocket rtpSocket,
                                             RtpPacket rtpPacket,
                                             RtpH264 rtpH264) {

        do {
            receiveRtpPacket(rtpSocket, rtpPacket, rtpH264);
            rtpH264.doProcess(rtpPacket);
        } while (!rtpH264.isConfigReady());

        MediaFormat format = MediaFormat.createVideoFormat("video/avc",
                                                           DEFAULT_WIDTH,
                                                           DEFAULT_HEIGHT);

        /*
         * Some decoders require the config information for the stream
         * before anything else.
         */
        format.setByteBuffer("csd-0", ByteBuffer.wrap(rtpH264.getSps()));
        format.setByteBuffer("csd-1", ByteBuffer.wrap(rtpH264.getPps()));
        return format;
    }
}
