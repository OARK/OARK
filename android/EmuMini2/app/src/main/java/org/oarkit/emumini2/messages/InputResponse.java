/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.emumini2.messages;

// The auto-generated classes from ROS.
import org.oarkit.emumini2.messages.rosmessages.Input;
import org.oarkit.emumini2.messages.rosmessages.InputRequestResponse;

/**
 * This is the response generated from the robot when the controller
 * queries for the inputs.
 */
public class InputResponse extends Message {
    // Internal OARK type for header.
    private final static byte OARK_TYPE = 3;

    /**
     * Takes in a response that's a result of an input query.
     */
    public InputResponse(byte[] inResponse) {
        super(InputRequestResponse._TYPE);
        setType(OARK_TYPE);

        fromByteArray(inResponse);
    }

    /**
     * Will return an array of Input objects.
     */
    public java.util.List<org.oarkit.emumini2.messages.rosmessages.Input> getValues() {
        InputRequestResponse message =
            (InputRequestResponse) this.getRosMessage();
        return message.getInputs();
    }
}
