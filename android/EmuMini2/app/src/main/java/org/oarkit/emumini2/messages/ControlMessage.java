/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.emumini2.messages;

// The auto-generated classes from ROS.
import org.oarkit.emumini2.messages.rosmessages.Command;

/**
 * Control messages are the ones that tell the robot to move it's
 * motors, it is essentially a matrix of floats that represent all the
 * controls of the Android controller.
 *
 * The order is specific, and it is not the responsibility of this
 * class to ensure that the mapping is correct.
 */
public class ControlMessage extends Message {
    // Internal OARK type for header.
    public final static byte OARK_TYPE = 1;
    private final static int FLOAT_SIZE = 4;

    /**
     * Creates an empty control message.
     */
    public ControlMessage() {
        super(Command._TYPE);
        setType(OARK_TYPE);
    }

    /**
     * Creates a control message with the given float array as the
     * values.
     *
     * @param inValues float array of values for the message.
     * @exception IllegalArgumentException if the array is too big for
     *                                     a message.
     */
    public ControlMessage(float[] inValues) throws IllegalArgumentException {
        this();
        setValues(inValues);
    }

    /**
     * Sets the values of the message to the given array of floats.
     *
     * @param inValues float array of values for the message.
     * @exception IllegalArgumentException if the array is too big for
     *                                     a message.
     */
    public void setValues(float[] inValues) throws IllegalArgumentException {
        if ((inValues.length * 4) > MAX_MESSAGE_SIZE) {
            throw new IllegalArgumentException("Message is too big. " +
                                               inValues.length * 4 + " > " +
                                               MAX_MESSAGE_SIZE);
        }

        Command message = (Command) this.getRosMessage();
        message.setValues(inValues);
    }

    public float[] getValues() {
        Command message = (Command) this.getRosMessage();
        return message.getValues();
    }
}
