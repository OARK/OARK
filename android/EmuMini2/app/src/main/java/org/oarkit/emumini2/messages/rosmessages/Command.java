package org.oarkit.emumini2.messages.rosmessages;

public interface Command extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "org/oarkit/emumini2/rosmessages/Command";
  static final java.lang.String _DEFINITION = "float32[] values\n";
  float[] getValues();
  void setValues(float[] value);
}
