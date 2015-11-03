package org.oarkit.controller.messages.rosmessages;

public interface InputRequestResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "org/oarkit/emumini2/messages/rosmessages/InputRequestResponse";
  static final java.lang.String _DEFINITION = "Input[] inputs";
  java.util.List<Input> getInputs();
  void setInputs(java.util.List<Input> value);
}
