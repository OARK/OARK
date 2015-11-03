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
public interface InputRequestResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "org/oarkit/controller/messages/rosmessages/InputRequestResponse";
  static final java.lang.String _DEFINITION = "Input[] inputs";
  java.util.List<Input> getInputs();
  void setInputs(java.util.List<Input> value);
}
