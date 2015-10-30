/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.emumini2.networking;

import org.oarkit.emumini2.messages.Message;

/**
 * This is an interface for anything that wants to register with the
 * Transceiver thread for when a message comes in from the network.
 *
 * It's up to the callback to determine if the message is the correct
 * type.
 */
public interface INetworkCallback {
    public void handleMessage(byte[] inMessage);
}
