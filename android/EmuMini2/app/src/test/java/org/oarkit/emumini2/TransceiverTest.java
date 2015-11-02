/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.emumini2;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.oarkit.emumini2.networking.INetworkCallback;

import static org.hamcrest.Matchers.is;
import static org.junit.Assert.assertThat;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.net.ServerSocket;
import java.net.Socket;

/**
 * Unit tests for the network messages to and from the controller.
 *
 * Be sure to add pauses to account for socket communications.
 *
 * This is a quick and bunch of units tests, they should be
 * refactored.
 */
public class TransceiverTest {
    private final static int TEST_PORT = 7777;
    private final static int PAUSE_TIME = 200;

    private ServerSocket mTestServerSocket;
    private Socket mTestClientSocket;

    private DataOutputStream mToTransceiver;
    private DataInputStream mFromTransceiver;

    private Transceiver mTransceiver;

    private byte[] mCurrentInMessage;

    private int mCallbackCount;

    /**
     * Test that a single message exactly in from the network is correct.
     */
    @Test
    public void TransceiverTest_singleExactMessageReceived() throws Exception {
        // Push out the most basic message.
        byte[] testMessage = new byte[]{1, 0, 0};

        mToTransceiver.write(testMessage);

        Thread.sleep(PAUSE_TIME);

        // Check the message.
        assertThat(mCurrentInMessage[0], is(testMessage[0]));
        assertThat(mCurrentInMessage[1], is(testMessage[1]));
        assertThat(mCurrentInMessage[2], is(testMessage[2]));

        assertThat(mCallbackCount, is(1));
    }

    /**
     * Test that if we receive half a message, then the other half, we
     * just get one complete message.
     */
    @Test
    public void TransceiverTest_halfAndHalf() throws Exception {
        byte[] testMessage = new byte[]{1, 0, 5, 1, 2, 3, 4, 5};

        mToTransceiver.write(testMessage, 0, testMessage.length / 2);
        Thread.sleep(PAUSE_TIME);
        assertThat(mCallbackCount, is(0));

        mToTransceiver.write(testMessage, (testMessage.length / 2),
                             testMessage.length - (testMessage.length / 2));

        Thread.sleep(PAUSE_TIME);
        assertThat(mCallbackCount, is(1));
    }

    /**
     * Test that if we receive half a message, then the other half,
     * plus a half of another message that we get a single message.
     * And when receive the remaining half, that message gets
     * received.
     */
    @Test
    public void TransceiverTest_halfAndHalfWithHalf() throws Exception {
        byte[] testMessageOne = new byte[]{1, 0, 5, 1, 2, 3, 4, 5};
        final int firstHalfMessageOne = testMessageOne.length / 2;

        mToTransceiver.write(testMessageOne, 0, firstHalfMessageOne);
        Thread.sleep(PAUSE_TIME);

        byte[] testMessageTwo = new byte[]{1, 0, 5, 6, 7, 8, 9, 10};
        final int firstHalfMessageTwo = testMessageTwo.length / 2;

        byte[] combinedMessage = new byte[testMessageOne.length -
                                          firstHalfMessageOne +
                                          firstHalfMessageTwo];

        System.arraycopy(testMessageOne, firstHalfMessageOne, combinedMessage,
                         0, testMessageOne.length - firstHalfMessageOne);
        System.arraycopy(testMessageTwo, 0, combinedMessage,
                         testMessageOne.length - firstHalfMessageTwo,
                         firstHalfMessageTwo);

        mToTransceiver.write(combinedMessage);
        Thread.sleep(PAUSE_TIME);

        assertThat(mCallbackCount, is(1));

        mToTransceiver.write(testMessageTwo, firstHalfMessageTwo,
                             testMessageTwo.length - firstHalfMessageTwo);

        Thread.sleep(PAUSE_TIME);

        assertThat(mCallbackCount, is(2));
    }

    /**
     * Receiving two complete messages at once.
     *
     * This should just result in one message being parsed, with the
     * other remaining in the buffer until data is received again.
     *
     * This behaviour needs to change in the future.
     */
    @Test
    public void TransceiverTest_twoComplete() throws Exception {
        byte[] testMessageOne = new byte[]{1, 0, 5, 5, 4, 3, 2, 1};
        byte[] testMessageTwo = new byte[]{1, 0, 5, 6, 7, 8, 9, 10};

        byte[] combinedMessage = new byte[testMessageOne.length +
                                          testMessageTwo.length];

        System.arraycopy(testMessageOne, 0, combinedMessage, 0,
                         testMessageOne.length);

        System.arraycopy(testMessageTwo, 0, combinedMessage,
                         testMessageOne.length, testMessageTwo.length);

        mToTransceiver.write(combinedMessage);

        Thread.sleep(PAUSE_TIME);

        assertThat(mCallbackCount, is(1));

        // Check the message.
        assertThat(mCurrentInMessage[0], is(testMessageOne[0]));
        assertThat(mCurrentInMessage[1], is(testMessageOne[1]));
        assertThat(mCurrentInMessage[2], is(testMessageOne[2]));
    }

    /**
     * Set up a local socket and get the Transceiver to connect to it.
     */
    @Before
    public void setUp() throws Exception {
        mCallbackCount = 0;

        mCurrentInMessage = new byte[0];

        mTestServerSocket = new ServerSocket(TEST_PORT);
        startTransceiver();
        setupClientSocket();
    }

    @After
    public void tearDown() throws Exception {
        mFromTransceiver.close();
        mToTransceiver.close();
        mTestClientSocket.close();
        mTestServerSocket.close();
    }

    /**
     * Set up client socket.
     */
    private void setupClientSocket() throws Exception {
        // Wait until we get a client connecting.
        mTestClientSocket = mTestServerSocket.accept();

        mToTransceiver =
            new DataOutputStream(mTestClientSocket.getOutputStream());
        mFromTransceiver =
            new DataInputStream(mTestClientSocket.getInputStream());
    }

    /**
     * Start the transceiver.
     *
     * This is done in a separate thread, after a delay, to make sure
     * the server thread is ready and waiting for the connect.
     */
    private void startTransceiver() {
        TransceiverTestThread delayTransceiver = new TransceiverTestThread();
        Thread delayedThread = new Thread(delayTransceiver);
        delayedThread.start();
    }

    class TransceiverTestThread implements Runnable {
        public void run() {
            try {
                Thread.sleep(PAUSE_TIME);
                mTransceiver = new Transceiver("localhost", TEST_PORT);
                mTransceiver.addCallback(new ReceiveTransceiverMessage());
                mTransceiver.start();
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }
    }

    class ReceiveTransceiverMessage implements INetworkCallback {
        public void handleMessage(byte[] inMessage) {
            mCallbackCount++;
            mCurrentInMessage = inMessage.clone();
        }
    }
}
