/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.controller.networking;

import android.os.AsyncTask;
import android.util.Log;

import org.oarkit.controller.Transceiver;
import org.oarkit.controller.messages.InputRequestMessage;
import org.oarkit.controller.messages.InputResponse;

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
        ConnectRobotException {
        InputRequestMessage requestInputs = new InputRequestMessage();
        InputResponse inputResponse;

        inTransceiver[0].start();
        inTransceiver[0].send(requestInputs);

        boolean responseReceived;
        byte[] tempBuffer;

        do {
            inTransceiver[0].readFromNetwork();

            int messageLength = inTransceiver[0].getMessageLength();
            tempBuffer = new byte[messageLength];
            responseReceived = inTransceiver[0].parseMessage(tempBuffer,messageLength);
            inputResponse = new InputResponse(tempBuffer);
        } while (!responseReceived);

        Log.i("InputsConfigTask", "Message in buffer. Type: " +
              tempBuffer[0]);

        Log.i("InputsConfigTask", "Response: " +
              inputResponse.toString());

        // Wait for response.
        return inputResponse;
    }

    @Override
    protected void onPostExecute(InputResponse inResponse) {
        if (callback != null) {
            callback.updateInputControllers(inResponse.getValues());
        } else {
           Log.e("InputsConfigTask", "No callback set");
        }
    }
}
