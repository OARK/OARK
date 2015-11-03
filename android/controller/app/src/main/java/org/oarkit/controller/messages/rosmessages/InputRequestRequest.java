/*
 * OARK Controller Software.
 *
 * Copyright (c) 2015 Open Academic Robot Kit.
 */

package org.oarkit.controller.messages.rosmessages;

/**
 * This interface represents the ROS data structure that needs to be
 * sent to the robot in order to request the available inputs need to
 * control that robot.
 */
public interface InputRequestRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "org/oarkit/controller/messages/rosmessages/InputRequestRequest";
  static final java.lang.String _DEFINITION = "";
}
