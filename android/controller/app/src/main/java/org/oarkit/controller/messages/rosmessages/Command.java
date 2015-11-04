/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.controller.messages.rosmessages;

/**
 * This is an interface for using the rosjava library for serialising
 * to and from ROS data structures.
 */
public interface Command extends org.ros.internal.message.Message {
    java.lang.String _TYPE = "org/oarkit/controller/messages/rosmessages/Command";
    java.lang.String _DEFINITION = "float32[] values\n";
    float[] getValues();
    void setValues(float[] value);
}
