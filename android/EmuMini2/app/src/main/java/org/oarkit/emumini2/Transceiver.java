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

/*
 * This class is the central point for communication to and from the
 * robot.
 */
public class Transceiver {
    private final int DEFAULT_PORT = 1717;
    private final int MAX_MESSAGE_SIZE = 1024;

    private Socket mRobotSocket;
    private DataOutputStream mToRobot;
    private DataInputStream mFromRobot;

    private Thread mTransceiverThread;

    private String mAddress;

    private Transceiver() {}

    public Transceiver(String address) throws IOException {
        mAddress = address;

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

        public void run() {
            try {
                mRobotSocket = new Socket(InetAddress.getByName(mAddress),
                                          DEFAULT_PORT);
                mToRobot =
                    new DataOutputStream(mRobotSocket.getOutputStream());
                mFromRobot =
                    new DataInputStream(mRobotSocket.getInputStream());

            } catch (IOException e) {
                Log.i("TransceiverThread", "IOException: " + e.getMessage());
            }
        }

        public Message getMessage() {
            try {
                mFromRobot.read(mMessageBytes, 0, mMessageBytes.length);
            } catch (IOException e) {
                Log.e("TransceiverThread", "IOException: " + e.getMessage());
            }

            int type = mMessageBytes[0];
            int length = mMessageBytes[1];

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
        ControlMessage testMessage = new ControlMessage(new float[]{0f, 0f, 0f, 0.1f, 0.5f, 0f, 0f});
        mToRobot.write(inMessage.toByteArray());
        mToRobot.flush();
    }
}
