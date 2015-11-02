/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.net.ServerSocket;
import java.net.Socket;

public class FakeRobot {
    private final static int TEST_PORT = 1717;
    private final static int MAX_MESSAGE_SIZE = 0xFFFF;

    private static ServerSocket testServerSocket;
    private static Socket testClientSocket;
    private static DataOutputStream toController;
    private static DataInputStream fromController;

    public static void main(String args[]) {
        setupSocket();

        byte[] dataBuffer = new byte[MAX_MESSAGE_SIZE];

        do {
            try {
                int bytesRead = fromController.read(dataBuffer);
                switch (dataBuffer[0]) {
                case 1:
                    break;
                case 2:
                    System.out.println("Received Input Request");
                    byte[] response = new byte[]{
                        0x03, 0x00, 0x26, 0x01, 0x00, 0x00, 0x00, 0x04,
                        0x00, 0x00, 0x00, 0x6e, 0x61, 0x6d, 0x65, 0x06,
                        0x00, 0x00, 0x00, 0x73, 0x6c, 0x69, 0x64, 0x65,
                        0x72, 0x04, 0x00, 0x00, 0x00, 0x74, 0x79, 0x70,
                        0x65, 0x04, 0x00, 0x00, 0x00, 0x61, 0x78, 0x65,
                        0x73
                    };

                    toController.write(response);
                    toController.flush();
                    break;
                default:
                    System.out.println("Unknown message type");
                }
            } catch (Exception e) {
                System.out.println("Main Loop, exception: " + e.getMessage());
                tearDownSocket();
                setupSocket();
            }
        } while (true);
    }

    private static void setupSocket() {
        try {
            testServerSocket = new ServerSocket(TEST_PORT);
            testClientSocket = testServerSocket.accept();

            toController =
                new DataOutputStream(testClientSocket.getOutputStream());
            fromController =
                new DataInputStream(testClientSocket.getInputStream());
        } catch (Exception e) {
            tearDownSocket();
            System.out.println("Exception: " + e.getMessage());
            System.exit(1);
        }
    }

    private static void tearDownSocket() {
        try {
            if (fromController != null) {
                fromController.close();
            }

            if (toController != null) {
                toController.close();
            }

            if (testClientSocket != null) {
                testClientSocket.close();
            }

            if (testServerSocket != null) {
                testServerSocket.close();
            }
        } catch (Exception e) {
            System.out.println("tearDownSocket, exception: " + e.getMessage());
        }
    }
}
