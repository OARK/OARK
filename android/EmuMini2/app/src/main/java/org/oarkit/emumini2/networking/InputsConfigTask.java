/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.emumini2.networking;

import android.os.AsyncTask;
import android.util.Log;

import org.oarkit.emumini2.Transceiver;
import org.oarkit.emumini2.messages.InputRequestMessage;
import org.oarkit.emumini2.messages.InputResponse;
import org.oarkit.emumini2.messages.rosmessages.Input;
import org.oarkit.emumini2.ui.ControllerTable;

import java.io.IOException;
import java.net.ConnectException;

/**
 * This is the task run for the app to query the robot for the
 * controls that should be made available to the user.
 */
public class InputsConfigTask extends AsyncTask<Transceiver, String, InputResponse> {
    private IInputRequestCallback callback;

    /**
     * Set the callback to be called when we have the response from the robot.
     */
    public void setCallback(IInputRequestCallback inCallback) {
        callback = inCallback;
    }

    /**
     * Setup a socket, send a request to the robot for available
     * inputs.
     *
     * The Transceiver passed in must be configured, but not started.
     */
    @Override
    protected InputResponse doInBackground(Transceiver... inTransceiver) throws
        RuntimeException {
        InputRequestMessage requestInputs = new InputRequestMessage();
        InputResponse inputResponse;

        try {
            inTransceiver[0].start();
            inTransceiver[0].send(requestInputs);

            boolean responseReceived = false;
            byte[] tempBuffer;

            do {
                inTransceiver[0].readFromNetwork();

                int messageLength = inTransceiver[0].getMessageLength();
                tempBuffer = new byte[messageLength];
                inputResponse = new InputResponse(tempBuffer);
                responseReceived = inTransceiver[0].parseMessage(tempBuffer,messageLength);
            } while (!responseReceived);

            Log.i("InputsConfigTask", "Message in buffer. Type: " +
                  tempBuffer[0]);

            Log.i("InputsConfigTask", "Response: " +
                  inputResponse.toString());

        } catch (ConnectException e) {
            Log.e("InputConfigTask", "Unable to connect to robot.");
            throw new RuntimeException(e);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        // Wait for response.
        return inputResponse;
    }

    @Override
    protected void onPostExecute(InputResponse inResponse) {
        callback.updateInputControllers(inResponse.getValues());
    }
}
