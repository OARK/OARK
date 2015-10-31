/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.emumini2.networking;

import org.oarkit.emumini2.messages.rosmessages.Input;

import java.util.List;

public interface IInputRequestCallback {
    public void updateInputControllers(List<Input> inInputs);
}
