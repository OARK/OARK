/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.emumini2;

import java.io.DataOutputStream;
import java.io.IOException;

/**
 * Simple interface for Msg classes to reduce coupling a
 * little bit.
 */
public interface IMsg {
    public int getSize();
    public void serialise(DataOutputStream dos) throws IOException;

}
