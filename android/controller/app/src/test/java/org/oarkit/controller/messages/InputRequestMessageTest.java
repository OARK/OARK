/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.controller.messages;

import org.junit.Test;

import static org.hamcrest.Matchers.is;
import static org.junit.Assert.assertThat;

/**
 * Unit tests for input requests to the robot.
 */
public class InputRequestMessageTest {
    /**
     * Test that we have a correctly formed InputRequest message.
     */
    @Test
    public void InputRequestMessage_request() {
        InputRequestMessage testMessage = new InputRequestMessage();

        byte[] testMessageArray = testMessage.toByteArray();

        assertThat(testMessageArray.length,
                   is(InputRequestMessage.HEADER_SIZE));

        assertThat(testMessageArray[0], is((byte) 2));
        assertThat(testMessageArray[1], is((byte) 0));
        assertThat(testMessageArray[2], is((byte) 0));
    }
}
