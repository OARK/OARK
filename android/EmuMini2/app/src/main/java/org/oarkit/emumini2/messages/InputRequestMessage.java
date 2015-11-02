/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.emumini2.messages;

// The auto-generated classes from ROS.
import org.oarkit.emumini2.messages.rosmessages.InputRequestRequest;

/**
 * Input request messages are used to request the array of inputs
 * available for the robot controls.
 *
 * Nothing needs to be set, as it's just the request itself that
 * triggers the information being sent.
 */
public class InputRequestMessage extends Message {
    // Internal OARK type for header.
    public final static byte OARK_TYPE = 2;

    /**
     * Create the input request message.
     */
    public InputRequestMessage() {
        super(InputRequestRequest._TYPE);
        setType(OARK_TYPE);
    }
}
