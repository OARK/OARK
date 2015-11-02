/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.emumini2.messages;

import org.junit.Test;

import static org.hamcrest.Matchers.is;
import static org.junit.Assert.assertThat;

import org.oarkit.emumini2.messages.rosmessages.Input;

import java.util.List;

public class InputResponseTest {
    private static byte[] testPacket = new byte[]{
        0x03, 0x00, 0x25, 0x01, 0x00, 0x00, 0x00, 0x04,
        0x00, 0x00, 0x00, 0x6e, 0x61, 0x6d, 0x65, 0x05,
        0x00, 0x00, 0x00, 0x74, 0x69, 0x74, 0x6c, 0x65,
        0x04, 0x00, 0x00, 0x00, 0x74, 0x79, 0x70, 0x65,
        0x04, 0x00, 0x00, 0x00, 0x61, 0x78, 0x65, 0x73
    };

    /**
     * Test creation of the input response.
     */
    @Test
    public void InputResponseTest_create() {
        InputResponse testResponse = new InputResponse(testPacket);

        List<Input> inputList = testResponse.getValues();

        // Single input only.
        Input firstInput = inputList.get(0);

        // Title and type in the test data are swapped around.
        assertThat(firstInput.getName(), is("name"));
        assertThat(firstInput.getType(), is("title"));
        assertThat(firstInput.getTitle(), is("type"));
        assertThat(firstInput.getAxes(), is("axes"));
    }
}
