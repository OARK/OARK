/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.controller.networking;

import org.oarkit.controller.messages.rosmessages.Input;

import java.util.List;

public interface IInputRequestCallback {
    public void updateInputControllers(List<Input> inInputs);
}
