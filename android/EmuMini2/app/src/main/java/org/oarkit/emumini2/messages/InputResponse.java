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
    public final static byte OARK_TYPE = 3;

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

    @Override
    public String toString() {
        String result = "";

        result += super.toString() + "\n";

        for (Input inp : getValues()) {
            result += (inp.getName().isEmpty() ? "" : "Name: " +
                       inp.getName()) +
                (inp.getType().isEmpty() ? "" : ", Type: " + inp.getType()) +
                (inp.getTitle().isEmpty() ? "" : ", Title: " + inp.getTitle()) +
                (inp.getAxes().isEmpty() ? "" : ", Axes: " + inp.getAxes()) +
                "\n";
        }

        return result;
    }
}
