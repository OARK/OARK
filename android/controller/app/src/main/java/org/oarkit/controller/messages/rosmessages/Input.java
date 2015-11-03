/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.controller.messages.rosmessages;

/**
 * This interface is for serialising the ROS data structure from the
 * robot that represents available inputs.
 */
public interface Input extends org.ros.internal.message.Message {
    static final java.lang.String _TYPE = "org/oarkit/controller/messages/rosmessages/Input";
    static final java.lang.String _DEFINITION = "string name\nstring type\nstring title\nstring axes\n";
    java.lang.String getName();
    void setName(java.lang.String value);
    java.lang.String getType();
    void setType(java.lang.String value);
    java.lang.String getTitle();
    void setTitle(java.lang.String value);
    java.lang.String getAxes();
    void setAxes(java.lang.String value);
}
