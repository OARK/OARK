/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.emumini2;

import android.util.Log;

import org.oarkit.emumini2.messages.Message;
import org.oarkit.emumini2.networking.INetworkCallback;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.InetAddress;
import java.net.Socket;
import java.util.Arrays;

/**
 * This class is the central receiver and transmitter of packets from
 * the robot.
 */
public class Transceiver {
    private final static int DEFAULT_PORT = 1717;

    private Socket mRobotSocket;
    private DataOutputStream mToRobot;
    private DataInputStream mFromRobot;

    private Thread mTransceiverThread;

    private String mAddress;
    private int mPort;

    // If the read thread should be running.
    private boolean mRunning;

    // Total number of bytes read off network since last message.
    private int mBytesRead;

    // Just support a single callback, we don't need more than one at
    // the moment.
    private INetworkCallback mCallback;

    private byte[] mMessageBuffer = new byte[Message.MAX_MESSAGE_SIZE];

    private Transceiver() {};

    /**
     * Given an IP address of the robot, connect to the robot on the
     * default port.
     */
    public Transceiver(String address) throws IOException {
        this(address, DEFAULT_PORT);
    }

    public Transceiver(String address, int port) throws IOException {
        mAddress = address;
        mPort = port;
        mBytesRead = 0;
        mRunning = false;
    }

    /**
     * Starts the transceiver thread.
     */
    public void start() {
        TransceiverThread ti = new TransceiverThread();
        mTransceiverThread = new Thread(ti);
        mTransceiverThread.start();
    }

    /**
     * Add a callback that gets called when we have a complete
     * message from the robot. Note: With the current
     * implementation only one callback is allowed, another call
     * of this method will overwrite any previous callback.
     *
     * @param inCallback Callback to be triggered when a full
     *                   message is received.
     */
    public void addCallback(INetworkCallback inCallback) {
        mCallback = inCallback;
    }

    /**
     * Because of the blocking nature of sockets, to receive we need a
     * thread that will sit and wait for data from the network.
     *
     * Anything that is interested in this data will have to register
     * with this class and will be called and passed in the data.
     *
     * The only data we are taking care of is Message data from the
     * robot, so it will wait until it has a complete packet, and pass
     * it the complete message.
     *
     * It's the responsibility of the method being called to cast the
     * Message into the correct class for processing.
     */
    class TransceiverThread implements Runnable {
        /**
         * Will try to open the socket with the port and address set
         * in the parent class.
         *
         * @exception RuntimeException Thrown when unable to open the
         *                             socket.
         */
        public TransceiverThread() throws RuntimeException {
            try {
                mRobotSocket = new Socket(InetAddress.getByName(mAddress),
                                          mPort);
                mToRobot =
                    new DataOutputStream(mRobotSocket.getOutputStream());
                mFromRobot =
                    new DataInputStream(mRobotSocket.getInputStream());

            } catch (IOException e) {
                Log.i("TransceiverThread", "IOException: " + e.getMessage());
                throw new RuntimeException(e);
            }
        }

        /**
         * Run forever until told to stop.
         *
         * It's expected that a callback has been added that will be
         * triggered when a full message is received from the robot.
         *
         * @exception IllegalStateException If executed without a
         *                                  callback being set.
         *
         * TODO: This scheme is not error resistant, it assumes that
         * it will receive nothing but OARK messages, it doesn't
         * verify that the data received is an OARK message.
         */
        public void run() throws IllegalStateException, RuntimeException {
            if (mCallback == null) {
                throw new IllegalStateException("No callback set");
            }

            mRunning = false;

            while (mRunning) {
                readFromNetwork();
                int messageLength = getMessageLength();
                byte[] tempBuffer = new byte[messageLength];

                if (parseMessage(tempBuffer, messageLength)) {
                    mCallback.handleMessage(tempBuffer);
                }
            }
        }
    }

    // Dirty hack for refactoring purposes.
    // Remove this!!!!!!!
    public DataInputStream getInputStream() {
        return mFromRobot;
    }

    public void readFromNetwork() {
        try {
            mBytesRead += mFromRobot.read(mMessageBuffer, mBytesRead,
                                          Message.MAX_MESSAGE_SIZE -
                                          mBytesRead);

            Log.i("Transceiver", "Total bytes Read so far: " + mBytesRead);

            if (mBytesRead > 0) {
                String testString = "";
                for (int i = 0; i < mBytesRead; i++) {
                    testString += String.valueOf(mMessageBuffer[i]) + " ";
                }

                Log.i("Transceiver", "Buffer: " + testString);
            }
        } catch (IOException e) {
            Log.e("Transceiver", "Exception: " + e.getMessage());
            throw new RuntimeException(e);
        }
    }

    public int getMessageLength() {
        int result = (((mMessageBuffer[1] << 8) & 0xFF00) |
                      (mMessageBuffer[2] & 0xFF));

        return result + Message.HEADER_SIZE;
    }

    /**
     * We've received something from the robot. Parse the message.
     *
     * Refactoring hack so can start cleaning up.
     */
    public boolean parseMessage(byte[] inTempBuffer, int inMessageLength) {
        // If we haven't got a full message, then keep waiting
        // until we do. Always keep the header of a message at
        // the start of the Message Buffer.

        // Test that header is within range of header types here?
        // How to do cleanly?

        boolean result = false;

        Log.i("Transceiver", "parseMessage");
        Log.i("Transceiver", "inMessageLength: " + inMessageLength + " " +
              "mBytesRead: " + mBytesRead);
        if (inMessageLength <= mBytesRead) {
            Log.i("Transceiver", "Message shorter, equal bytes read.");
            result = true;

            // We've got the message (and maybe more) just
            // copy the message bytes, leaving the rest as the
            // next message.
            System.arraycopy(mMessageBuffer, 0, inTempBuffer, 0, inMessageLength);

            final int remainingBytesLength = mMessageBuffer.length -
                inMessageLength;

            // Shuffled any remaining bytes down to the start
            // of the array.
            byte[] tmp = new byte[remainingBytesLength];

            System.arraycopy(mMessageBuffer, inMessageLength, tmp, 0,
                             remainingBytesLength);

            System.arraycopy(tmp, 0, mMessageBuffer, 0,
                             remainingBytesLength);

            mBytesRead = remainingBytesLength;
        }

        return result;
    }

    /*
     * Close off the streams and sockets.
     */
    public void close() throws IOException {
        mFromRobot.close();
        mToRobot.close();
        mRobotSocket.close();
    }

    /*
     *
     */
    public synchronized void send(Message inMessage) throws IOException {
        Log.i("Transceiver", "Message: " + inMessage.toString());
        mToRobot.write(inMessage.toByteArray());
        mToRobot.flush();
    }
}
