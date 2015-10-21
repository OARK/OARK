package org.oarkit.emumini2;


/*
 * Class for the encapsulated message from over the network.
 *
 * TYPE, LENGTH, MESSAGE
 *
 * Should be encoded into the relevant ROS type.
 */
public class Message {
    private byte mType;
    private byte mLength;
    private byte[] mMessage[0];

    private Message() {}

    /*
     * Given a byte array, convert into the necessary message type.
     */
    public Message(byte[] inMessage) {
    }
}
