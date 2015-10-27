/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.emumini2.messages;

import org.junit.Test;

import static org.hamcrest.Matchers.is;
import static org.junit.Assert.assertThat;

/**
 * Unit tests for control messages to the robot.
 */
public class ControlMessageTest {
    // ROS has it's own length counter for the number of elements in
    // message.
    private final static int ROS_LENGTH_SIZE = 4;
    private final static int FLOAT_SIZE = 4;

    /**
     * Test that an empty ControlMessage just has the header and the
     * length of zero for the ROS data.
     */
    @Test
    public void ControlMessageTest_emptyMessage() {
        // Empty message should just be a header.
        ControlMessage testMessage = new ControlMessage();

        // The ROS serialise code will write four bytes, even if
        // passed empty message. (Assuming it things single float
        // value of 0)
        assertThat(testMessage.toByteArray().length,
                   is(ControlMessage.HEADER_SIZE + ROS_LENGTH_SIZE));
    }

    /**
     * Test single float value for ControlMessage.
     */
    @Test
    public void ControlMessageTest_singleFloat() {
        float[] testValues = new float[]{(float)0xffffffff};

        ControlMessage testMessage =
            new ControlMessage(testValues);

        assertThat(testMessage.toByteArray().length,
                   is(ControlMessage.HEADER_SIZE +
                      (testValues.length * FLOAT_SIZE) +
                      ROS_LENGTH_SIZE));
    }

    /**
     * Check that a message with the maximum size allowed is generated
     * correctly.
     */
    //@Test
    public void ControlMessageTest_maxSize() {
        final int MAX_SIZE = (ControlMessage.MAX_MESSAGE_SIZE /
                              FLOAT_SIZE) - ROS_LENGTH_SIZE;
        float[] testValues = new float[MAX_SIZE];

        ControlMessage testMessage =
            new ControlMessage(testValues);

        byte[] testMessageArray = testMessage.toByteArray();

        assertThat(testMessageArray.length,
                   is(ControlMessage.HEADER_SIZE +
                      (testValues.length * FLOAT_SIZE) +
                      ROS_LENGTH_SIZE));

        int packetSize = MAX_SIZE * FLOAT_SIZE;

        assertThat(testMessageArray[0], is((byte) 1));
        assertThat(testMessageArray[1], is((byte) ((packetSize & 0xFF00)
                                                   >> 8)));
        assertThat(testMessageArray[2], is((byte) ((packetSize & 0xFF))));
    }

    /**
     * Test parsing of deserialize packets.
     */
    @Test
    public void ControlMessageTest_packetDeserialize() {
        float[] testValues = new float[]{1.0f, 1.0f};

        ControlMessage testMessage = new ControlMessage(testValues);

        byte[] outMessageArray = testMessage.toByteArray();

        ControlMessage testReceive = new ControlMessage();
        testReceive.fromByteArray(outMessageArray);

        assertThat(testValues, is(testReceive.getValues()));
    }
}
