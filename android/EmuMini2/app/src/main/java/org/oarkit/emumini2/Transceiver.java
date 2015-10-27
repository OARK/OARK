/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.emumini2;

import android.util.Log;

import org.oarkit.emumini2.messages.ControlMessage;
import org.oarkit.emumini2.messages.Message;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.InetAddress;
import java.net.Socket;

/**
 * This class is the central receiver and transmitter of packets from
 * the robot.
 */
public class Transceiver {
    private final static int DEFAULT_PORT = 1717;
    private final static int MAX_MESSAGE_SIZE = 1024;

    private Socket mRobotSocket;
    private DataOutputStream mToRobot;
    private DataInputStream mFromRobot;

    private Thread mTransceiverThread;

    private String mAddress;
    private int mPort;

    private Transceiver() {};

    public Transceiver(String address) throws IOException {
        this(address, DEFAULT_PORT);
    }

    public Transceiver(String address, int port) throws IOException {
        mAddress = address;
        mPort = port;

        createAndStartThread();
    }

    private void createAndStartThread() {
        TransceiverThread ti = new TransceiverThread();
        mTransceiverThread = new Thread(ti);
        mTransceiverThread.start();
    }

    /*
     * We have to have a separate thread outside the UI for socket
     * communications.
     */
    class TransceiverThread implements Runnable {
        private byte[] mMessageBytes = new byte[MAX_MESSAGE_SIZE];

        public TransceiverThread() {
        }

        // We throw a RuntimeException here if the connection fails.
        // TODO: Improve error handling so it will keep retrying and
        // application won't proceed unless the connection is working.
        public void run() {
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

        public Message getMessage() {
            try {
                mFromRobot.read(mMessageBytes, 0, mMessageBytes.length);
            } catch (IOException e) {
                Log.e("TransceiverThread", "IOException: " + e.getMessage());
            }

            int type = mMessageBytes[0];
            int length = (mMessageBytes[1] << 8) | mMessageBytes[2];

            if ((mMessageBytes.length - 3) == length) {
                Log.i("TransceiverThread", "Length is correct");
            }

            return null;
        }
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
